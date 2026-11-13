"""Sharpa measured joint trajectories -> Wuji Hand 2 early Beta, offline only.

The output floating root is expressed in each source Sharpa root frame, NOT in
the camera/world frame. Default outputs include the user-confirmed local-Z
correction (left +90 degrees, right -90 degrees) after geometric IK.
No arm IK or robot connection is performed.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
os.environ.setdefault('MUJOCO_GL','egl')
try:
    import mujoco
except ImportError:
    sys.path.append('/home/descfly/miniforge3/envs/mujoco-env/lib/python3.11/site-packages')
    import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares

HERE=Path(__file__).resolve().parent
TIPS=np.array([4,8,12,16,20]); PARENTS=np.array([0]+[v for f in range(5) for v in [0,1+4*f,2+4*f,3+4*f]])
SHARPA=['thumb_CMC_FE','thumb_CMC_AA','thumb_MCP_FE','thumb_MCP_AA','thumb_IP']+[f'{f}_{j}' for f in ['index','middle','ring'] for j in ['MCP_FE','MCP_AA','PIP','DIP']]+['pinky_CMC','pinky_MCP_FE','pinky_MCP_AA','pinky_PIP','pinky_DIP']
WUJI=['thumb_cmc_flex','thumb_cmc_abd','thumb_mcp','thumb_ip']+[f'{f}_{j}' for f in ['index_finger','middle_finger','ring_finger','pinky'] for j in ['mcp_flex','mcp_abd','pip','dip']]
ALIGNMENT_VERSION='wuji_beta_local_z_left_p90_right_m90_v1'
ALIGNMENT_DEG={'left':90.,'right':-90.}


def stat(a):
    a=np.asarray(a);return dict(mean=float(a.mean()),rmse=float(np.sqrt(np.mean(a*a))),p95=float(np.percentile(a,95)),max=float(a.max()))


def pose_attrs(origin):
    if origin is None:return {}
    r=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_quat()[[3,0,1,2]]
    return dict(pos=origin.get('xyz','0 0 0'),quat=' '.join(map(str,r)))


class Hand:
    def __init__(self,kind,side,visual=False):
        self.kind,self.side=kind,side
        self.path=HERE/'assets'/('sharpa' if kind=='sharpa' else 'wuji/urdf')/f'{side}.urdf'
        self.urdf=ET.parse(self.path).getroot()
        self.joints=list(self.urdf.findall('joint'))
        children={j.find('child').get('link') for j in self.joints}
        self.root=next(l.get('name') for l in self.urdf.findall('link') if l.get('name') not in children)
        if kind=='sharpa':
            self.names=[f'{side}_{n}' for n in SHARPA]
            self.points=[self.root]
            for f in ['thumb','index','middle','ring','pinky']:
                self.points += [f'{side}_{f}_{s}' for s in (['CMC_VL','MCP_VL','DP','fingertip'] if f=='thumb' else ['MCP_VL','MP','DP','fingertip'])]
        else:
            p=side[0];self.names=[f'{p}_{n}' for n in WUJI];self.points=[f'{p}_wrist']
            for f in ['thumb','index_finger','middle_finger','ring_finger','pinky']:
                self.points += [f'{p}_{f}_{s}' for s in ['proximal_abd','middle','distal','tip']]
        jdict={j.get('name'):j for j in self.joints}
        self.lo=np.array([float(jdict[n].find('limit').get('lower')) for n in self.names]);self.hi=np.array([float(jdict[n].find('limit').get('upper')) for n in self.names])
        self.vel=np.array([float(jdict[n].find('limit').get('velocity')) for n in self.names]);self.neutral=np.clip(np.zeros(len(self.names)),self.lo,self.hi)
        doc=ET.Element('mujoco',model=f'{kind}_{side}');ET.SubElement(doc,'compiler',angle='radian',autolimits='true',fusestatic='false')
        ET.SubElement(doc,'option',gravity='0 0 0');vis=ET.SubElement(doc,'visual');ET.SubElement(vis,'global',offwidth='1280',offheight='1000');ET.SubElement(vis,'headlight',ambient='.5 .5 .5',diffuse='.7 .7 .7',specular='.1 .1 .1')
        asset=ET.SubElement(doc,'asset');world=ET.SubElement(doc,'worldbody');bodies={self.root:ET.SubElement(world,'body',name=self.root)}
        pending=list(self.joints)
        while pending:
            before=len(pending)
            for j in pending[:]:
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in bodies:continue
                b=ET.SubElement(bodies[parent],'body',name=child,**pose_attrs(j.find('origin')));bodies[child]=b
                ET.SubElement(b,'inertial',mass='.01',pos='0 0 0',diaginertia='.00001 .00001 .00001')
                if j.get('type')!='fixed':
                    assert j.get('type')=='revolute';lim=j.find('limit')
                    ET.SubElement(b,'joint',name=j.get('name'),axis=j.find('axis').get('xyz'),range=lim.get('lower')+' '+lim.get('upper'))
                pending.remove(j)
            assert len(pending)<before
        for i,name in enumerate(self.points):ET.SubElement(bodies[name],'site',name=f'point{i}',size='.001',rgba='1 .5 0 1')
        if visual:
            if kind=='sharpa':
                # Use the previously validated renderer's exact URDF body transforms and visual fallbacks.
                old=ET.parse(HERE/'assets/sharpa/original_bimanual.xml').getroot()
                for mesh in old.findall('asset/mesh'):
                    m=copy.deepcopy(mesh);m.set('file',str(HERE/'assets/sharpa/meshes'/Path(m.get('file')).name));asset.append(m)
                for ob in old.findall('.//body'):
                    if ob.get('name') in bodies:
                        for geom in ob.findall('geom'):bodies[ob.get('name')].append(copy.deepcopy(geom))
            else:
                for link in self.urdf.findall('link'):
                    for v in link.findall('visual'):
                        mesh=v.find('geometry/mesh')
                        if mesh is None:continue
                        filename=(self.path.parent/mesh.get('filename')).resolve();assert filename.exists()
                        name='mesh'+str(len(asset));ET.SubElement(asset,'mesh',name=name,file=str(filename))
                        ET.SubElement(bodies[link.get('name')],'geom',type='mesh',mesh=name,contype='0',conaffinity='0',rgba='.65 .78 .86 1',**pose_attrs(v.find('origin')))
        self.xml=ET.tostring(doc).decode();self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model)
        self.qids=np.array([self.model.joint(n).qposadr[0] for n in self.names]);self.dofs=np.array([self.model.joint(n).dofadr[0] for n in self.names]);self.siteids=np.array([self.model.site(f'point{i}').id for i in range(21)])

    def fk(self,q,jac=False):
        self.data.qpos[self.qids]=q;mujoco.mj_forward(self.model,self.data)
        points=self.data.site_xpos[self.siteids].copy()
        if not jac:return points
        jj=[]
        for sid in self.siteids:
            a=np.zeros((3,self.model.nv));mujoco.mj_jacSite(self.model,self.data,a,None,sid);jj.append(a[:,self.dofs])
        return points,np.array(jj)

    def independent(self,qs):
        n=len(qs);trans={self.root:np.tile(np.eye(4),(n,1,1))};qmap=dict(zip(self.names,qs.T));pending=list(self.joints)
        while pending:
            before=len(pending)
            for j in pending[:]:
                par=j.find('parent').get('link')
                if par not in trans:continue
                o=j.find('origin');a=np.eye(4)
                if o is not None:a[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');a[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
                b=np.tile(np.eye(4),(n,1,1))
                if j.get('type')!='fixed':b[:,:3,:3]=Rotation.from_rotvec(qmap[j.get('name')][:,None]*np.fromstring(j.find('axis').get('xyz'),sep=' ')).as_matrix()
                trans[j.find('child').get('link')]=trans[par]@a@b;pending.remove(j)
            assert len(pending)<before
        return np.stack([trans[x][:,:3,3] for x in self.points],axis=1),trans


def rigid_align(a,b):
    u,_,vh=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)));fix=np.eye(3);fix[2,2]=np.linalg.det(vh.T@u.T);r=vh.T@fix@u.T
    return r,b.mean(0)-r@a.mean(0)


def solve(source, target, frames):
    p0=target.fk(target.neutral);s0=source.fk(source.neutral)
    r,t=rigid_align(p0[[1,5,9,13,17]],s0[[1,5,9,13,17]])
    prev=np.r_[target.neutral,Rotation.from_matrix(r).as_rotvec(),t]
    w=np.full(21,.05);w[0]=.1;w[TIPS]=1.;w=np.sqrt(w)[:,None]
    bounds=(np.r_[target.lo,np.full(6,-np.inf)],np.r_[target.hi,np.full(6,np.inf)])
    result=[];error=[];evals=[];status=[];optimality=[]
    for i,goal in enumerate(frames):
        previous_q=prev[:20].copy()
        temporal=1e-4 if i==0 else .005
        def fun(z):
            rot=Rotation.from_rotvec(z[20:23]).as_matrix();p=target.fk(z[:20])@rot.T+z[23:26]
            return np.r_[((p-goal)*w).ravel(),temporal*(z[:20]-previous_q)]
        def jac(z):
            rot=Rotation.from_rotvec(z[20:23]).as_matrix();p,j=target.fk(z[:20],True);out=np.zeros((83,26))
            out[:63,:20]=(np.einsum('ij,njk->nik',rot,j)*w[:,:,None]).reshape(63,20)
            for k in range(3):
                d=np.eye(3)[k]*1e-6;dr=(Rotation.from_rotvec(z[20:23]+d).as_matrix()-Rotation.from_rotvec(z[20:23]-d).as_matrix())/(2e-6)
                out[:63,20+k]=(p@dr.T*w).ravel()
            out[:63,23:]=np.tile(np.eye(3),(21,1))*np.repeat(w,3,axis=0);out[63:,:20]=temporal*np.eye(20)
            return out
        prev[:20]=np.clip(prev[:20],target.lo+1e-9,target.hi-1e-9)
        sol=least_squares(fun,prev,jac=jac,bounds=bounds,max_nfev=300,ftol=1e-9,xtol=1e-9,gtol=1e-8)
        prev=sol.x.copy();pred=target.fk(prev[:20])@Rotation.from_rotvec(prev[20:23]).as_matrix().T+prev[23:]
        result.append(prev.copy());error.append(np.linalg.norm(pred-goal,axis=-1));evals.append(sol.nfev);status.append(sol.status);optimality.append(sol.optimality)
        if i%200==0:print(f'{source.side} IK {i}/{len(frames)} tips {np.mean(error[-1][TIPS])*1000:.2f}mm',flush=True)
    return np.array(result),np.array(error),dict(nfev=evals,status=status,optimality=optimality,initial_alignment_rotation=r.tolist(),initial_alignment_translation=t.tolist())


def convert(episode):
    z=np.load(episode/'source_sharpa.npz');out=dict(time_s=z['time_s'],timestamp_ns=z['timestamp_ns'],rgb_index=z['rgb_index'],side_order=np.array(['left','right']))
    report=dict(method='weighted absolute 21-point geometric IK, free wrist, no length scaling',target='official early hand2_beta; same URDF bytes as local OpenPI referenced control stack',root_frame='each source Sharpa hand_C_MC frame; left/right have distinct local frames',weights=dict(tips=1,wrist=.1,intermediate=.05,joint_temporal=2.5e-5,first_frame_joint_temporal=1e-8),frames=len(z['time_s']),fps=30,sides={})
    for side in ['left','right']:
        src=Hand('sharpa',side);dst=Hand('wuji',side);source_names=list(z[f'{side}_hand_names']);q=z[f'{side}_hand_q'][:,[source_names.index(n) for n in src.names]]
        points=np.array([src.fk(a) for a in q]);sol,err,diag=solve(src,dst,points)
        q2=sol[:,:20];rot=Rotation.from_rotvec(sol[:,20:23]).as_matrix();pos=sol[:,23:]
        local=np.array([dst.fk(a) for a in q2]);pred=np.einsum('nij,nkj->nki',rot,local)+pos[:,None,:]
        independent,links=dst.independent(q2);source_ind,_=src.independent(q)
        independent_max=float(abs(independent-local).max());source_max=float(abs(source_ind-points).max())
        assert independent_max<1e-9 and source_max<1e-9
        limit=np.maximum(np.maximum(dst.lo-q2,q2-dst.hi),0);assert limit.max()<1e-9 and np.isfinite(sol).all()
        steps=abs(np.diff(q2,axis=0));speed=steps/np.diff(z['time_s'])[:,None]
        # Compare every link, every frame between independent URDF matrix chains and MuJoCo.
        full_link_error=0.
        for fi,row in enumerate(q2):
            dst.fk(row)
            for name,tf in links.items():
                b=dst.model.body(name).id
                full_link_error=max(full_link_error,float(abs(tf[fi,:3,3]-dst.data.xpos[b]).max()),float(abs(tf[fi,:3,:3]-dst.data.xmat[b].reshape(3,3)).max()))
        assert full_link_error<1e-8
        report['sides'][side]=dict(tip_error_mm=stat(err[:,TIPS]*1000),all_point_error_mm=stat(err*1000),per_finger_tip_mean_mm=(err[:,TIPS].mean(0)*1000).tolist(),joint_step_deg=stat(np.rad2deg(steps)),joint_speed_max_rad_s=float(speed.max()),urdf_speed_exceedances=int((speed>dst.vel).sum()),joint_limit_violation_max_rad=float(limit.max()),frames_at_joint_limit=int(np.any((abs(q2-dst.lo)<1e-5)|(abs(q2-dst.hi)<1e-5),axis=1).sum()),source_joint_limit_violation_max_rad=float(np.maximum(np.maximum(src.lo-q,q-src.hi),0).max()),wrist_point_error_mm=stat(err[:,0]*1000),root_step_mm=stat(np.linalg.norm(np.diff(pos,axis=0),axis=-1)*1000),root_rotation_step_deg=stat(np.rad2deg(Rotation.from_matrix(rot[1:]@rot[:-1].transpose(0,2,1)).magnitude())),independent_fk_max_m=independent_max,source_independent_fk_max_m=source_max,all_link_transform_max_abs=full_link_error,solver_budget_exhausted_frames=int(np.count_nonzero(np.array(diag['status'])==0)),max_nfev=max(diag['nfev']))
        out.update({f'{side}_sharpa_joints_rad':q,f'{side}_sharpa_joint_names':np.array(src.names),f'{side}_wuji_joints_rad':q2,f'{side}_wuji_joint_names':np.array(dst.names),f'{side}_root_xyz_in_sharpa_m':pos,f'{side}_root_rotvec_in_sharpa_rad':sol[:,20:23],f'{side}_source_points_m':points,f'{side}_wuji_points_in_sharpa_m':pred,f'{side}_error_m':err,f'{side}_wuji_action_local':np.c_[pos,sol[:,20:23],q2],f'{side}_solver_nfev':np.array(diag['nfev']),f'{side}_solver_status':np.array(diag['status'])})
        (episode/f'{side}_solver.json').write_text(json.dumps(diag,indent=2))
    report['checks']=dict(finite=True,within_joint_position_limits=True,independent_urdf_mujoco_agree=True)
    apply_confirmed_alignment(out,report)
    save_conversion(episode,out,report)


def apply_confirmed_alignment(out,report):
    """Apply the user-confirmed local-base rotation exactly once.

    This changes physical poses, not just their coordinate representation.
    Keep the geometric IK solution separately so repeat migrations cannot
    accumulate rotations and its fitting residuals remain auditable.
    """
    existing=out.get('alignment_version')
    if existing is not None and str(existing)!=ALIGNMENT_VERSION:
        raise ValueError(f'Unsupported existing alignment: {existing}')
    for side,angle in ALIGNMENT_DEG.items():
        keys=['root_rotvec_in_sharpa_rad','wuji_points_in_sharpa_m','error_m']
        if existing is None:
            for key in keys:
                out[f'{side}_unaligned_{key}']=out[f'{side}_{key}'].copy()
        for key in keys:
            if f'{side}_unaligned_{key}' not in out:
                raise ValueError(f'Missing original IK field: {side}_{key}')
        original_rotation=Rotation.from_rotvec(out[f'{side}_unaligned_root_rotvec_in_sharpa_rad']).as_matrix()
        correction=Rotation.from_euler('z',angle,degrees=True).as_matrix()
        rotation=original_rotation@correction
        position=out[f'{side}_root_xyz_in_sharpa_m']
        joints=out[f'{side}_wuji_joints_rad']
        local,_=Hand('wuji',side).independent(joints)
        predicted=np.einsum('nij,nkj->nki',rotation,local)+position[:,None,:]
        original_points=out[f'{side}_unaligned_wuji_points_in_sharpa_m']
        relative_rotation=rotation@original_rotation.transpose(0,2,1)
        rotated_original=np.einsum('nij,nkj->nki',relative_rotation,original_points-position[:,None,:])+position[:,None,:]
        assert np.max(abs(predicted-rotated_original))<1e-8
        errors=np.linalg.norm(predicted-out[f'{side}_source_points_m'],axis=-1)
        rotvec=Rotation.from_matrix(rotation).as_rotvec()
        out.update({f'{side}_root_rotvec_in_sharpa_rad':rotvec,
                    f'{side}_wuji_points_in_sharpa_m':predicted,
                    f'{side}_error_m':errors,
                    f'{side}_wuji_action_local':np.c_[position,rotvec,joints]})
        stats=report['sides'][side]
        before=out[f'{side}_unaligned_error_m']
        stats['pre_alignment_fit']=dict(tip_error_mm=stat(before[:,TIPS]*1000),all_point_error_mm=stat(before*1000),wrist_point_error_mm=stat(before[:,0]*1000))
        stats.update(tip_error_mm=stat(errors[:,TIPS]*1000),all_point_error_mm=stat(errors*1000),
                     per_finger_tip_mean_mm=(errors[:,TIPS].mean(0)*1000).tolist(),
                     wrist_point_error_mm=stat(errors[:,0]*1000),
                     root_rotation_step_deg=stat(np.rad2deg(Rotation.from_matrix(rotation[1:]@rotation[:-1].transpose(0,2,1)).magnitude())),
                     alignment_independent_fk_max_m=float(abs(predicted-rotated_original).max()))
    out['alignment_version']=np.array(ALIGNMENT_VERSION)
    out['alignment_local_z_deg_left_right']=np.array([ALIGNMENT_DEG[s] for s in ['left','right']])
    report['alignment']=dict(version=ALIGNMENT_VERSION,confirmed_date='2026-11-10',local_z_deg=ALIGNMENT_DEG,
        composition='R_corrected = R_ik @ Rz(angle); p_corrected = p_ik; hand joints unchanged',
        pivot='each Wuji base origin; clockwise viewed from local +Z toward origin',
        applied_count=1,scope='user-confirmed visual orientation; physical flange mounting and target-arm IK remain unresolved')
    report['method']='weighted absolute 21-point geometric IK followed by user-confirmed local-Z orientation correction'
    report['error_reference']='Current errors compare corrected Wuji points with unchanged source Sharpa points. Solver residuals are in pre_alignment_fit and unaligned_* arrays.'
    if 'continuity_comparison' in report:
        report['continuity_comparison_scope']='pre-alignment geometric IK; not corrected-output fitting error'
    report.pop('video_validation',None)


def save_conversion(episode,out,report):
    np.savez_compressed(episode/'wuji_trajectory.npz',**out)
    np.save(episode/'wuji_hand_joints_left_right.npy',np.c_[out['left_wuji_joints_rad'],out['right_wuji_joints_rad']])
    np.save(episode/'wuji_actions_local_right_left.npy',np.c_[out['right_wuji_action_local'],out['left_wuji_action_local']])
    export_tables(episode,out)
    (episode/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)


def update_alignment(episode):
    with np.load(episode/'wuji_trajectory.npz') as data:
        out={key:data[key].copy() for key in data.files}
    report=json.loads((episode/'validation.json').read_text())
    apply_confirmed_alignment(out,report)
    save_conversion(episode,out,report)


def export_tables(episode,out):
    joint_order={side:[str(x) for x in out[f'{side}_wuji_joint_names']] for side in ['left','right']}
    (episode/'joint_order.json').write_text(json.dumps(joint_order,indent=2))
    columns=[out['time_s']]+[out[f'{side}_error_m'][:,tip]*1000 for side in ['left','right'] for tip in TIPS]
    np.savetxt(episode/'fingertip_errors_mm.csv',np.column_stack(columns),delimiter=',',comments='',header='time_s,'+','.join(f'{side}_{f}' for side in ['left','right'] for f in ['thumb','index','middle','ring','pinky']))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('episode',type=Path)
    ap.add_argument('--update-alignment',action='store_true',help='Update an existing conversion from its saved original IK solution; safe to repeat.')
    args=ap.parse_args()
    if args.update_alignment:update_alignment(args.episode)
    else:convert(args.episode)
