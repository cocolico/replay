"""Read equivalent camera arrays from individual NPY files or a Wuji NPZ package."""
from pathlib import Path
import numpy as np

FIELDS = ('time_s', 'timestamp_ns', 'frame', 'pose_target', 'quaternion_order',
          'position_unit', 'joint_unit', 'alignment_version') + tuple(
    side + '_' + name for side in ('left', 'right') for name in
    ('root_pose', 'position_m', 'quaternion_wxyz', 'joint_names', 'joint_positions_rad'))


def read_camera_input(directory):
    root = Path(directory).resolve()
    if not root.is_dir():
        raise ValueError('请选择轨迹文件夹：' + str(root))
    if (root / 'time_s.npy').is_file():
        missing = [name + '.npy' for name in FIELDS if not (root / (name + '.npy')).is_file()]
        if missing:
            raise ValueError('缺少轨迹文件：' + ', '.join(missing))
        arrays = {name: np.load(root / (name + '.npy'), allow_pickle=False) for name in FIELDS}
        sources = sorted(root.glob('*.npy'))
        kind = 'npy_directory'
    else:
        archive = root / 'wuji2_camera_replay_trajectory.npz'
        if not archive.is_file():
            if (root / 'sharpa_camera_replay_trajectory.npz').is_file():
                raise ValueError('该目录是Sharpa轨迹（22关节/Sharpa掌根），请选择对应的wuji目录；不能直接作为Wuji轨迹执行。')
            raise ValueError('未找到分开的NPY轨迹或 wuji2_camera_replay_trajectory.npz')
        with np.load(archive, allow_pickle=False) as data:
            missing = [name for name in FIELDS if name not in data.files]
            if missing:
                raise ValueError('NPZ缺少轨迹字段：' + ', '.join(missing))
            arrays = {name: data[name].copy() for name in FIELDS}
        sources = [archive]
        kind = 'wuji_npz'
    label = str(arrays['pose_target'])
    if 'l_base_link' not in label or 'r_base_link' not in label:
        raise ValueError('轨迹掌根不是Wuji l_base_link/r_base_link；需要先转换手模型。')
    t = arrays['time_s']
    if t.ndim != 1 or len(t) < 2 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('轨迹时间轴必须有限、递增且至少两帧')
    for side in ('left', 'right'):
        for name, shape in (('joint_positions_rad', (len(t), 20)), ('joint_names', (20,)),
                            ('root_pose', (len(t), 4, 4)), ('position_m', (len(t), 3)),
                            ('quaternion_wxyz', (len(t), 4))):
            array = arrays[side + '_' + name]
            if array.shape != shape:
                raise ValueError(f'{side}_{name} 维度错误：{array.shape}，要求 {shape}')
            if name != 'joint_names' and not np.isfinite(array).all():
                raise ValueError(side + '_' + name + ' 包含非有限值')
    return arrays, sources, kind
