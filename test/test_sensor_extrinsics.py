from pathlib import Path

from jackal_nav2.sensor_extrinsics import (
    load_zed_extrinsics,
    zed_extrinsics_are_calibrated,
)

import pytest


def write_config(path: Path, zed_body: str) -> Path:
    path.write_text(f"zed:\n{zed_body}", encoding="utf-8")
    return path


def valid_body(calibrated: str = "false") -> str:
    return (
        "  x: 0.25\n"
        "  y: 0\n"
        "  z: 0.50\n"
        "  roll: 0\n"
        "  pitch: 0.0\n"
        "  yaw: -0.1\n"
        f"  calibrated: {calibrated}\n"
    )


def test_load_zed_extrinsics(tmp_path: Path):
    config = write_config(tmp_path / "extrinsics.yaml", valid_body())
    extrinsics = load_zed_extrinsics(config)

    assert extrinsics.transform_values() == (0.25, 0.0, 0.5, 0.0, 0.0, -0.1)
    assert not extrinsics.calibrated
    assert not zed_extrinsics_are_calibrated(config)


def test_calibration_check_accepts_true(tmp_path: Path):
    config = write_config(tmp_path / "extrinsics.yaml", valid_body("true"))
    assert zed_extrinsics_are_calibrated(config)


@pytest.mark.parametrize("value", ["missing", ".nan", ".inf", "true", "text"])
def test_reject_missing_or_non_finite_transform_value(tmp_path: Path, value: str):
    body = valid_body()
    if value == "missing":
        body = body.replace("  pitch: 0.0\n", "")
    else:
        body = body.replace("  pitch: 0.0", f"  pitch: {value}")
    config = write_config(tmp_path / "extrinsics.yaml", body)

    with pytest.raises(ValueError):
        load_zed_extrinsics(config)


def test_reject_non_boolean_calibration_state(tmp_path: Path):
    config = write_config(tmp_path / "extrinsics.yaml", valid_body("0"))
    with pytest.raises(ValueError, match="zed.calibrated must be a boolean"):
        load_zed_extrinsics(config)
