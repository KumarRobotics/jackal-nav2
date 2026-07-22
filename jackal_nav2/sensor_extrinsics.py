"""Validated sensor-extrinsics configuration helpers."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Tuple, Union

import yaml


PathLike = Union[str, Path]
TRANSFORM_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


@dataclass(frozen=True)
class ZedExtrinsics:
    """Rigid transform from the configured base frame to the ZED link frame."""

    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    calibrated: bool

    def transform_values(self) -> Tuple[float, float, float, float, float, float]:
        """Return translation and roll/pitch/yaw in launch argument order."""
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


def _finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"zed.{field} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"zed.{field} must be a finite number")
    return converted


def load_zed_extrinsics(path: PathLike) -> ZedExtrinsics:
    """Load and validate the ZED transform from a sensor-extrinsics YAML file."""
    config_path = Path(path).expanduser()
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict):
        raise ValueError("Sensor extrinsics file must contain a YAML mapping")

    zed = document.get("zed")
    if not isinstance(zed, dict):
        raise ValueError("Sensor extrinsics file must contain a 'zed' mapping")

    required_fields = (*TRANSFORM_FIELDS, "calibrated")
    missing = [field for field in required_fields if field not in zed]
    if missing:
        raise ValueError(f"zed extrinsics missing required fields: {', '.join(missing)}")

    calibrated = zed["calibrated"]
    if not isinstance(calibrated, bool):
        raise ValueError("zed.calibrated must be a boolean")

    values = {field: _finite_number(zed[field], field) for field in TRANSFORM_FIELDS}
    return ZedExtrinsics(**values, calibrated=calibrated)


def zed_extrinsics_are_calibrated(path: PathLike) -> bool:
    """Validate an extrinsics file and report whether its ZED transform is calibrated."""
    return load_zed_extrinsics(path).calibrated
