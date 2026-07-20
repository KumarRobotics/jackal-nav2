from pathlib import Path

from jackal_nav2.goto_nav2 import load_waypoints_from_yaml, yaw_to_quaternion

import pytest


def test_load_mapping_waypoints(tmp_path: Path):
    path = tmp_path / "waypoints.yaml"
    path.write_text(
        "frame_id: map\nwaypoints:\n  - {x: 1, y: 2, yaw: 0.5}\n",
        encoding="utf-8",
    )
    frame, waypoints = load_waypoints_from_yaml(str(path))
    assert frame == "map"
    assert waypoints == [(1.0, 2.0, 0.5)]


def test_reject_empty_waypoints(tmp_path: Path):
    path = tmp_path / "waypoints.yaml"
    path.write_text("frame_id: map\nwaypoints: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_waypoints_from_yaml(str(path))


def test_yaw_to_quaternion():
    assert yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)
