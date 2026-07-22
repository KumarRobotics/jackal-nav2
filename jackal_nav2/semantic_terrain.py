"""Pure processing primitives for SAM3-backed semantic terrain observations."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Generic, Protocol, Sequence, TypeVar

from jackal_nav2.semantic_terrain_sampling import (
    invalidate_xyz_outside_depth_range,
    resample_aligned,
)

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


DEFAULT_PROMPTS = (
    "paved road",
    "compact gravel",
    "firm dirt",
    "short grass",
    "loose gravel",
    "tall vegetation",
    "mud",
    "deep sand",
    "curb",
    "water",
    "ditch",
    "drop-off",
)
DEFAULT_PROMPT_CLASS_IDS = (1, 1, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4)
DEFAULT_PROMPT_CLASS_NAMES = (
    "preferred_surface",
    "preferred_surface",
    "preferred_surface",
    "caution_surface",
    "caution_surface",
    "high_risk_surface",
    "high_risk_surface",
    "high_risk_surface",
    "high_risk_surface",
    "water_or_dropoff",
    "water_or_dropoff",
    "water_or_dropoff",
)


@dataclass(frozen=True)
class PromptClassConfig:
    """Stable mapping from text prompts to costmap-facing category IDs and names."""

    prompts: tuple[str, ...]
    prompt_class_ids: tuple[int, ...]
    prompt_class_names: tuple[str, ...]

    @classmethod
    def from_sequences(
        cls,
        prompts: Sequence[str],
        prompt_class_ids: Sequence[int],
        prompt_class_names: Sequence[str],
    ) -> "PromptClassConfig":
        """Validate and normalize three parallel prompt configuration arrays."""
        if not prompts:
            raise ValueError("at least one semantic prompt is required")
        if len(prompts) != len(prompt_class_ids) or len(prompts) != len(
            prompt_class_names
        ):
            raise ValueError(
                "prompts, prompt_class_ids, and prompt_class_names must have equal lengths"
            )

        normalized_prompts = tuple(str(prompt).strip() for prompt in prompts)
        normalized_names = tuple(str(name).strip() for name in prompt_class_names)
        if any(not prompt for prompt in normalized_prompts):
            raise ValueError("semantic prompts must be non-empty strings")
        if any(not name for name in normalized_names):
            raise ValueError("semantic class names must be non-empty strings")
        if len(set(normalized_prompts)) != len(normalized_prompts):
            raise ValueError("semantic prompts must be unique")

        normalized_ids = []
        for class_id in prompt_class_ids:
            if isinstance(class_id, bool):
                raise ValueError("semantic class IDs must be integers, not booleans")
            class_id_int = int(class_id)
            if class_id_int != class_id:
                raise ValueError(f"semantic class ID {class_id!r} is not an integer")
            if class_id_int < 1 or class_id_int > 255:
                raise ValueError(
                    f"semantic class ID {class_id_int} is outside [1, 255]; 0 is reserved"
                )
            normalized_ids.append(class_id_int)

        id_to_name: dict[int, str] = {}
        name_to_id: dict[str, int] = {}
        for class_id, name in zip(normalized_ids, normalized_names):
            existing_name = id_to_name.setdefault(class_id, name)
            if existing_name != name:
                raise ValueError(
                    f"semantic class ID {class_id} maps to both {existing_name!r} and {name!r}"
                )
            existing_id = name_to_id.setdefault(name, class_id)
            if existing_id != class_id:
                raise ValueError(
                    f"semantic class name {name!r} maps to IDs {existing_id} and {class_id}"
                )

        return cls(normalized_prompts, tuple(normalized_ids), normalized_names)

    @property
    def unique_classes(self) -> tuple[tuple[int, str], ...]:
        """Return one ordered ``(class_id, class_name)`` pair per category."""
        classes: list[tuple[int, str]] = []
        seen: set[int] = set()
        for class_id, name in zip(self.prompt_class_ids, self.prompt_class_names):
            if class_id not in seen:
                classes.append((class_id, name))
                seen.add(class_id)
        return tuple(classes)


@dataclass(frozen=True)
class InstanceMask:
    """One SAM3 instance mask whose class index addresses the prompt array."""

    class_index: int
    score: float
    mask: np.ndarray


class SegmentationBackend(Protocol):
    """Minimal inference interface used by the ROS-independent processor."""

    def infer(self, bgr_image: np.ndarray, prompts: Sequence[str]) -> Sequence[InstanceMask]:
        """Return instance masks for ``bgr_image`` and the ordered prompts."""

    def close(self) -> None:
        """Release backend resources."""


@dataclass(frozen=True)
class SemanticFrame:
    """Pixel-aligned semantic and geometric products for one RGB-D frame."""

    label_mask: np.ndarray
    confidence: np.ndarray
    xyz: np.ndarray
    accepted_instance_count: int
    inference_seconds: float


def compose_semantic_maps(
    image_shape: tuple[int, int],
    instances: Sequence[InstanceMask],
    config: PromptClassConfig,
    score_threshold: float,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Fuse overlapping instances into mono8 class and confidence masks.

    The highest-scoring instance owns each pixel. Equal-score ties are resolved by
    prompt order, which makes the result stable for a stable configuration.
    """
    if len(image_shape) != 2 or image_shape[0] <= 0 or image_shape[1] <= 0:
        raise ValueError(f"invalid image shape {image_shape!r}")
    if not 0.0 <= score_threshold <= 1.0:
        raise ValueError("score_threshold must be in [0.0, 1.0]")

    height, width = image_shape
    label_mask = np.zeros((height, width), dtype=np.uint8)
    score_map = np.zeros((height, width), dtype=np.float32)
    accepted = 0

    # Prompt order, rather than detector return order, provides deterministic ties.
    ordered_instances = sorted(
        enumerate(instances), key=lambda item: (int(item[1].class_index), item[0])
    )
    for _, instance in ordered_instances:
        class_index = int(instance.class_index)
        if class_index < 0 or class_index >= len(config.prompts):
            raise ValueError(
                f"backend returned class index {class_index} for {len(config.prompts)} prompts"
            )
        score = float(instance.score)
        if not math.isfinite(score):
            continue
        if score < score_threshold:
            continue

        mask = np.asarray(instance.mask, dtype=bool)
        if mask.shape != (height, width):
            raise ValueError(
                f"instance mask shape {mask.shape} does not match image {(height, width)}"
            )
        update = mask & (score > score_map)
        label_mask[update] = config.prompt_class_ids[class_index]
        score_map[update] = score
        accepted += 1

    confidence = np.rint(np.clip(score_map, 0.0, 1.0) * 255.0).astype(np.uint8)
    return label_mask, confidence, accepted


def depth_to_xyz(
    depth: np.ndarray,
    camera_matrix: Sequence[float],
    *,
    integer_depth_scale: float = 0.001,
) -> np.ndarray:
    """Deproject a registered depth image into an organized optical-frame XYZ array."""
    depth_array = np.asarray(depth)
    if depth_array.ndim == 3 and depth_array.shape[2] == 1:
        depth_array = depth_array[:, :, 0]
    if depth_array.ndim != 2:
        raise ValueError(f"depth image must be two-dimensional, got {depth_array.shape}")
    if integer_depth_scale <= 0.0 or not math.isfinite(integer_depth_scale):
        raise ValueError("integer_depth_scale must be finite and positive")
    if len(camera_matrix) != 9:
        raise ValueError("CameraInfo.k must contain exactly 9 values")

    fx = float(camera_matrix[0])
    fy = float(camera_matrix[4])
    cx = float(camera_matrix[2])
    cy = float(camera_matrix[5])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError("camera intrinsics must be finite")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")

    if np.issubdtype(depth_array.dtype, np.integer):
        z = depth_array.astype(np.float32) * np.float32(integer_depth_scale)
    else:
        z = depth_array.astype(np.float32, copy=True)

    height, width = z.shape
    u, v = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    valid = np.isfinite(z) & (z > 0.0)
    xyz = np.full((height, width, 3), np.nan, dtype=np.float32)
    xyz[:, :, 0][valid] = (u[valid] - cx) * z[valid] / fx
    xyz[:, :, 1][valid] = (v[valid] - cy) * z[valid] / fy
    xyz[:, :, 2][valid] = z[valid]
    return xyz


class SemanticTerrainProcessor:
    """Run an injected segmentation backend and build aligned semantic products."""

    def __init__(
        self,
        backend: SegmentationBackend,
        config: PromptClassConfig,
        *,
        score_threshold: float,
        integer_depth_scale: float = 0.001,
        output_width: int = 320,
        output_height: int = 180,
        min_depth_m: float = 0.5,
        max_depth_m: float = 8.0,
    ):
        if not 0.0 <= score_threshold <= 1.0:
            raise ValueError("score_threshold must be in [0.0, 1.0]")
        if output_width <= 0 or output_height <= 0:
            raise ValueError("semantic output width and height must be positive")
        if not math.isfinite(min_depth_m) or min_depth_m < 0.0:
            raise ValueError("min_depth_m must be finite and non-negative")
        if not math.isfinite(max_depth_m) or max_depth_m <= min_depth_m:
            raise ValueError("max_depth_m must be finite and greater than min_depth_m")
        self.backend = backend
        self.config = config
        self.score_threshold = float(score_threshold)
        self.integer_depth_scale = float(integer_depth_scale)
        self.output_width = int(output_width)
        self.output_height = int(output_height)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)

    def process(
        self,
        bgr_image: np.ndarray,
        depth: np.ndarray,
        camera_matrix: Sequence[float],
    ) -> SemanticFrame:
        """Create class, confidence, and XYZ arrays with identical height and width."""
        bgr = np.asarray(bgr_image)
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f"BGR image must have shape (H, W, 3), got {bgr.shape}")
        if bgr.dtype != np.uint8:
            raise ValueError(f"BGR image must use uint8 pixels, got {bgr.dtype}")

        depth_array = np.asarray(depth)
        if depth_array.ndim == 3 and depth_array.shape[2] == 1:
            depth_array = depth_array[:, :, 0]
        if depth_array.shape != bgr.shape[:2]:
            raise ValueError(
                f"registered depth shape {depth_array.shape} does not match RGB {bgr.shape[:2]}"
            )

        inference_started = time.perf_counter()
        instances = self.backend.infer(bgr, self.config.prompts)
        inference_seconds = time.perf_counter() - inference_started
        labels, confidence, accepted = compose_semantic_maps(
            bgr.shape[:2], instances, self.config, self.score_threshold
        )
        xyz = depth_to_xyz(
            depth_array,
            camera_matrix,
            integer_depth_scale=self.integer_depth_scale,
        )
        xyz = invalidate_xyz_outside_depth_range(
            xyz,
            min_depth_m=self.min_depth_m,
            max_depth_m=self.max_depth_m,
        )
        labels, confidence, xyz = resample_aligned(
            (labels, confidence, xyz),
            output_height=self.output_height,
            output_width=self.output_width,
        )
        return SemanticFrame(
            labels,
            confidence,
            xyz,
            accepted,
            inference_seconds,
        )


def render_overlay(
    bgr_image: np.ndarray,
    label_mask: np.ndarray,
    *,
    alpha: float = 0.5,
) -> np.ndarray:
    """Render deterministic category colors over a BGR source image."""
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("overlay alpha must be in [0.0, 1.0]")
    image = np.asarray(bgr_image)
    labels = np.asarray(label_mask)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("overlay source must be a uint8 BGR image")
    if labels.shape != image.shape[:2]:
        raise ValueError("label mask and overlay source dimensions must match")

    output = image.copy()
    for class_id in np.unique(labels):
        if class_id == 0:
            continue
        color = np.random.default_rng(int(class_id)).integers(
            64, 256, size=3, dtype=np.uint8
        )
        selected = labels == class_id
        output[selected] = (
            (1.0 - alpha) * output[selected] + alpha * color
        ).astype(np.uint8)
    return output


def organized_pointcloud2(xyz: np.ndarray, header: Header) -> PointCloud2:
    """Serialize ``(H, W, 3)`` float XYZ data without losing organization."""
    points = np.asarray(xyz)
    if points.ndim != 3 or points.shape[2] != 3:
        raise ValueError(f"XYZ array must have shape (H, W, 3), got {points.shape}")
    points_le = np.ascontiguousarray(points, dtype="<f4")
    height, width, _ = points_le.shape

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = height
    cloud.width = width
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 12 * width
    cloud.is_dense = bool(np.isfinite(points_le).all())
    cloud.data = points_le.tobytes(order="C")
    return cloud


T = TypeVar("T")


class LatestFrameMailbox(Generic[T]):
    """A blocking single-slot mailbox where a newer frame replaces a stale one."""

    def __init__(self):
        self._condition = threading.Condition()
        self._pending: T | None = None
        self._closed = False
        self._dropped = 0

    @property
    def dropped(self) -> int:
        with self._condition:
            return self._dropped

    def put(self, value: T) -> bool:
        """Store ``value`` and return false only after the mailbox is closed."""
        with self._condition:
            if self._closed:
                return False
            if self._pending is not None:
                self._dropped += 1
            self._pending = value
            self._condition.notify()
            return True

    def take(self) -> T | None:
        """Block for a frame and return ``None`` once closed and drained."""
        with self._condition:
            while self._pending is None and not self._closed:
                self._condition.wait()
            value = self._pending
            self._pending = None
            return value

    def take_nowait(self) -> T | None:
        """Return and clear the current frame without blocking."""
        with self._condition:
            value = self._pending
            self._pending = None
            return value

    def note_discarded(self) -> None:
        """Count a frame already removed by a consumer but superseded before use."""
        with self._condition:
            self._dropped += 1

    def clear(self) -> None:
        """Discard a pending frame, if any."""
        with self._condition:
            if self._pending is not None:
                self._pending = None
                self._dropped += 1

    def close(self) -> None:
        """Wake consumers and reject subsequent frames."""
        with self._condition:
            self._closed = True
            self._pending = None
            self._condition.notify_all()
