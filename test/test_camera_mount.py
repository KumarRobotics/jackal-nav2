from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from jackal_nav2.camera_mount import static_arguments,mount_transform

CONFIG=Path(__file__).parents[1]/'config/zed_mount.yaml'


def test_calibrated_root_composes_exact_retained_optical_extrinsic():
    parent,child,t,meta=mount_transform(CONFIG)
    assert parent=='os_lidar' and child=='zed_camera_link'
    assert np.allclose(t@meta['root_to_optical'],meta['lidar_to_optical'],atol=1e-12)
    assert len(meta['calibration_sha256'])==64
    with pytest.raises(ValueError,match='optical frame'):static_arguments(CONFIG,child_frame='zed_left_camera_optical_frame')
    with pytest.raises(ValueError,match='LiDAR frame'):static_arguments(CONFIG,parent_frame='base_link')


def test_manual_angles_require_explicit_mode_and_invalid_geometry_rejected(tmp_path):
    c=yaml.safe_load(CONFIG.read_text());c['zed_mount'].update(mode='manual',pitch_rad=.1)
    path=tmp_path/'mount.yaml';path.write_text(yaml.safe_dump(c))
    parent,child,t,_=mount_transform(path,'body','camera_root')
    assert parent=='body' and child=='camera_root'
    assert Rotation.from_matrix(t[:3,:3]).as_euler('xyz')[1]==pytest.approx(.1)
    c['zed_mount']['x_m']=float('nan');path.write_text(yaml.safe_dump(c))
    with pytest.raises(ValueError,match='finite number'):static_arguments(path)
    with pytest.raises(ValueError,match='distinct'):static_arguments(CONFIG,'same','same')
