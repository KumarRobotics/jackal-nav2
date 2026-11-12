"""A ZED root edge derived from the same extrinsic used by LiDAR projection."""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def rigid(matrix):
    t=np.asarray(matrix,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
            or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3,:3]),1.,atol=1e-5)):
        raise ValueError('Mount calibration must contain a finite rigid transform')
    return t


def fixed_chain(urdf,parent,child):
    edges={}
    for joint in ET.fromstring(urdf).findall('joint'):
        if joint.get('type')!='fixed':continue
        origin=joint.find('origin');t=np.eye(4)
        if origin is not None:
            t[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
            t[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
        edges.setdefault(joint.find('parent').get('link'),[]).append((joint.find('child').get('link'),rigid(t)))
    stack=[(parent,np.eye(4))];seen=set()
    while stack:
        frame,t=stack.pop()
        if frame==child:return t
        if frame in seen:continue
        seen.add(frame)
        stack.extend((name,t@edge) for name,edge in edges.get(frame,[]))
    raise ValueError(f'No fixed ZED description chain from {parent} to {child}')


def mount_transform(config_file,parent_frame='',child_frame=''):
    c=yaml.safe_load(Path(config_file).read_text())['zed_mount']
    parent=parent_frame or c['parent_frame'];child=child_frame or c['child_frame']
    if not all(isinstance(f,str) and f and not f.startswith('/') for f in (parent,child)) or parent==child:
        raise ValueError('ZED mount requires distinct nonempty TF frame names without leading slashes')
    if child.endswith('_optical_frame'):
        raise ValueError('Attach the ZED root frame, not an optical frame already parented by the ZED description')
    if c['mode']=='calibration':
        import xacro
        from ament_index_python.packages import get_package_share_directory
        path=Path(c['calibration_file']);calibration=yaml.safe_load(path.read_text())
        if parent!=calibration['frames']['lidar']:
            raise ValueError('Calibration mount parent must be the calibration LiDAR frame; do not relabel it as base_link')
        description=Path(get_package_share_directory(c['description_package']))/c['description_relative_path']
        urdf=xacro.process_file(str(description),mappings={'camera_name':c['camera_name'],
            'camera_model':c['camera_model'],'custom_baseline':str(c['custom_baseline_m'])}).toxml()
        root_to_optical=fixed_chain(urdf,child,calibration['frames']['camera_optical'])
        lidar_to_optical=rigid(calibration['extrinsics']['T_lidar_camera'])
        transform=rigid(lidar_to_optical@np.linalg.inv(root_to_optical))
        metadata={'mode':'calibration','calibration_id':calibration['calibration_id'],
            'calibration_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'optical_frame':calibration['frames']['camera_optical'],
            'root_to_optical':root_to_optical.tolist(),'lidar_to_optical':lidar_to_optical.tolist()}
    elif c['mode']=='manual':
        values=[c[k] for k in ('x_m','y_m','z_m','roll_rad','pitch_rad','yaw_rad')]
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in values):
            raise ValueError('Manual mount values must be finite numbers')
        transform=np.eye(4);transform[:3,3]=values[:3];transform[:3,:3]=Rotation.from_euler('xyz',values[3:]).as_matrix()
        metadata={'mode':'manual'}
    else:raise ValueError('zed_mount.mode must be calibration or manual')
    return parent,child,transform,metadata


def static_arguments(config_file,parent_frame='',child_frame=''):
    parent,child,t,_=mount_transform(config_file,parent_frame,child_frame)
    q=Rotation.from_matrix(t[:3,:3]).as_quat()
    args=['--frame-id',parent,'--child-frame-id',child]
    for flag,value in zip(('x','y','z','qx','qy','qz','qw'),[*t[:3,3],*q]):args.extend(['--'+flag,str(float(value))])
    return args
