#!/usr/bin/env python3
"""Offline mesh preview of the repaired desired hand path. Never imports device adapters."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation, Slerp

from replay_pipeline import HERE, config, read_json, origin
from render_mount_preview import stl, reduce_mesh
from refine_hand_curve import interpolator


class MeshHand:
    def __init__(self, urdf, names, color):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.indices = [self.model.joints[self.model.getJointId(str(n))].idx_q for n in names]
        self.meshes = []
        for link in ET.parse(urdf).getroot().findall('link'):
            visual = link.find('visual')
            if visual is None or visual.find('geometry/mesh') is None:
                continue
            vertices, faces = reduce_mesh(stl(urdf.parent/visual.find('geometry/mesh').get('filename')), .002)
            rgb = (220, 129, 171) if 'thumb' in link.get('name') else color
            if 'base_link' in link.get('name'):
                rgb = (166, 180, 193)
            self.meshes.append((self.model.getFrameId(link.get('name')), origin(visual), vertices, faces, rgb))

    def triangles(self, q):
        positions = np.zeros(self.model.nq)
        positions[self.indices] = q
        pin.forwardKinematics(self.model, self.data, positions)
        pin.updateFramePlacements(self.model, self.data)
        tris, colors = [], []
        for frame, offset, vertices, faces, color in self.meshes:
            t = self.data.oMf[frame].homogeneous @ offset
            v = vertices@t[:3, :3].T+t[:3, 3]
            tris.append(v[faces])
            colors.append(np.tile(color, (len(faces), 1)))
        return np.concatenate(tris), np.concatenate(colors)


def camera(rect, center, eye_direction, width):
    center = np.asarray(center)
    forward = -np.asarray(eye_direction, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.]);right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return rect, center, np.stack([right, up, forward], axis=1), width


def project(vertices, view):
    (x, y, w, h), center, basis, width = view
    p = (vertices-center)@basis
    scale = w/width
    return np.stack([x+w/2+p[..., 0]*scale, y+h/2-p[..., 1]*scale], axis=-1), p[..., 2]


def draw_mesh(canvas, tris, colors, view):
    xy, depth = project(tris, view)
    normals = np.cross(tris[:, 1]-tris[:, 0], tris[:, 2]-tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = np.array([.3, -.5, .8]);light /= np.linalg.norm(light)
    intensity = .43+.55*np.abs(normals@light)
    bgr = np.clip(colors[:, ::-1]*intensity[:, None], 0, 255).astype(np.uint8)
    polygons = np.round(xy).astype(np.int32)
    x, y, w, h = view[0]
    keep = (polygons[:, :, 0].max(1)>=x)&(polygons[:, :, 0].min(1)<x+w)&(polygons[:, :, 1].max(1)>=y)&(polygons[:, :, 1].min(1)<y+h)
    for i in np.argsort(depth.mean(1))[::-1]:
        if keep[i]:
            cv2.fillConvexPoly(canvas, polygons[i], bgr[i].tolist(), lineType=cv2.LINE_AA)


def text(canvas, value, point, size=.65, color=(217, 225, 235)):
    cv2.putText(canvas, value, point, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--still', action='store_true')
    args = parser.parse_args()
    folder = HERE/'output/human_camera_1_repaired'
    c = config(HERE/'config/human_replay_1_repaired.json')
    curve = np.load(folder/'hand_curve.npz')
    prepared = np.load(folder/'human_camera_prepared.npz')
    board = np.asarray(read_json(c['current_table_observation'])['camera_T_board'])
    board_inv = np.linalg.inv(board)
    hands, splines, roots, rotations = {}, {}, {}, {}
    for side, color in [('left', (81, 179, 221)), ('right', (236, 177, 81))]:
        urdf = Path(c['source_episode_root'])/f'assets/wuji/urdf/{side}.urdf'
        hands[side] = MeshHand(urdf, curve[side+'_joint_names'], color)
        splines[side] = interpolator(curve['time_s'], curve[side+'_joint_positions_rad'])
        camera_T_base = np.linalg.inv(np.asarray(read_json(c['sides'][side]['handeye'])['T_base_camera']))
        roots[side] = board_inv@camera_T_base@prepared[side+'_aubo_base_T_wuji_base']
        rotations[side] = Slerp(prepared['time_s'], Rotation.from_matrix(roots[side][:, :3, :3]))
    flip = np.diag([1., 1., -1.])
    rootpoints = np.concatenate([r[:, :3, 3]@flip for r in roots.values()])
    lo, hi = rootpoints.min(0)-.16, rootpoints.max(0)+.16
    center = (lo+hi)/2
    globalview = camera((0, 80, 1280, 465), center, [.25, 1.2, 1.0], max(1.3, (hi-lo).max()*1.5))
    # Fit the complete path in this wide panel, including the fingertips.
    bounds_lo, bounds_hi = np.full(2, np.inf), np.full(2, -np.inf)
    basis = globalview[2]
    for at in curve['time_s']:
        for side in ('left', 'right'):
            triangles, _ = hands[side].triangles(splines[side](at))
            rotation = rotations[side](at).as_matrix()
            position = np.array([np.interp(at, prepared['time_s'], roots[side][:,j,3]) for j in range(3)])
            points = ((triangles.reshape(-1,3)@rotation.T+position)@flip-center)@basis
            bounds_lo = np.minimum(bounds_lo, points[:,:2].min(0))
            bounds_hi = np.maximum(bounds_hi, points[:,:2].max(0))
    offset = (bounds_lo+bounds_hi)/2
    center = center+basis[:,:2]@offset
    width = max((bounds_hi[0]-bounds_lo[0])*1.12, (bounds_hi[1]-bounds_lo[1])*1280/465*1.12)
    globalview = camera((0,80,1280,465), center, [.25,1.2,1.0], width)
    localviews = {'left':camera((0, 585, 640, 315), [0, 0, .11], [1., -.25, .15], .60),
                  'right':camera((640, 585, 640, 315), [0, 0, .11], [-1., -.25, .15], .60)}
    duration = float(curve['time_s'][-1]);fps = 20
    times = np.array([6.3]) if args.still else np.linspace(0, duration, int(np.ceil(duration*fps))+1)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    encoder = None
    if not args.still:
        encoder = subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','bgr24',
                                    '-video_size','1280x960','-framerate',str(fps),'-i','-', '-an','-c:v','libx264',
                                    '-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(args.out)], stdin=subprocess.PIPE)
    for index, t in enumerate(times):
        image = np.full((960, 1280, 3), (29, 24, 19), dtype=np.uint8)
        cv2.rectangle(image, (16, 80), (1264, 545), (43, 36, 29), -1)
        cv2.rectangle(image, (16, 585), (624, 900), (38, 31, 25), -1)
        cv2.rectangle(image, (656, 585), (1264, 900), (38, 31, 25), -1)
        text(image, 'WUJI / REPAIRED HUMAN TRAJECTORY', (28, 35), .85)
        text(image, 'Offline hand preview | arm replay remains blocked', (28, 63), .57, (154, 184, 210))
        text(image, f'{t:05.2f} / {duration:.2f} s   1x source time', (856, 40), .62)
        for grid in np.arange(-1., 1.61, .1):
            for line in [np.array([[grid,-.5,0],[grid,.8,0]]),np.array([[-1.,grid,0],[1.6,grid,0]])]:
                pixels,_=project(line,globalview)
                ok, a, b = cv2.clipLine((16,80,1248,465), tuple(np.round(pixels[0]).astype(int)),tuple(np.round(pixels[1]).astype(int)))
                if ok:cv2.line(image,a,b,(61,58,49),1,cv2.LINE_AA)
        alltris, allcolors, labels = [], [], []
        for side in ('left','right'):
            tri, colors = hands[side].triangles(splines[side](t))
            draw_mesh(image, tri, colors, localviews[side])
            transform = np.eye(4);transform[:3,:3] = rotations[side](t).as_matrix()
            for j in range(3):transform[j,3] = np.interp(t, prepared['time_s'], roots[side][:,j,3])
            globaltri = (tri@transform[:3,:3].T+transform[:3,3])@flip
            alltris.append(globaltri);allcolors.append(colors)
            labelpos, _ = project(transform[:3,3]@flip, globalview)
            labels.append((side.upper(), tuple(np.round(labelpos+[12,20]).astype(int)), (235,197,129) if side=='left' else (116,193,237)))
        draw_mesh(image, np.concatenate(alltris), np.concatenate(allcolors), globalview)
        for value, point, color in labels:text(image,value,point,.5,color)
        text(image, 'DESIRED HAND MOTION / table plane grid = 10 cm', (30, 108), .58)
        text(image, 'LEFT / finger detail (root held for inspection)', (28, 570), .57, (235, 197, 129))
        text(image, 'RIGHT / finger detail (root held for inspection)', (668, 570), .57, (116, 193, 237))
        text(image, '302 hand keys | roots preserved | display mesh simplified | no objects / arms', (28, 928), .54)
        cv2.rectangle(image, (28, 943), (1252, 949), (68, 61, 52), -1)
        cv2.rectangle(image, (28, 943), (28+int(1224*t/duration), 949), (130, 210, 114), -1)
        if args.still:
            cv2.imwrite(str(args.out), image)
        else:
            encoder.stdin.write(image.tobytes())
        if index%50==0:
            print('render',index,'/',len(times),flush=True)
    if encoder is not None:
        encoder.stdin.close()
        if encoder.wait()!=0:
            raise RuntimeError('Video encoding failed')


if __name__ == '__main__':
    main()
