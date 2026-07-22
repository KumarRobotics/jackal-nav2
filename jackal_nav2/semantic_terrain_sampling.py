"""Shared sampling and depth-range operations for aligned semantic products."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def nearest_neighbor_indices(input_size: int, output_size: int) -> np.ndarray:
    """Return deterministic OpenCV-style nearest-neighbor source indices."""
    if input_size <= 0 or output_size <= 0:
        raise ValueError("input and output dimensions must be positive")
    indices = np.floor(
        np.arange(output_size, dtype=np.float64) * input_size / output_size
    ).astype(np.intp)
    return np.minimum(indices, input_size - 1)


def resample_aligned(
    arrays: Sequence[np.ndarray],
    *,
    output_height: int,
    output_width: int,
) -> tuple[np.ndarray, ...]:
    """Nearest-neighbor sample aligned arrays with one shared pixel mapping."""
    if not arrays:
        raise ValueError("at least one aligned array is required")
    normalized = tuple(np.asarray(array) for array in arrays)
    input_shape = normalized[0].shape[:2]
    if len(input_shape) != 2 or input_shape[0] <= 0 or input_shape[1] <= 0:
        raise ValueError("aligned arrays must have non-empty height and width")
    for array in normalized:
        if array.ndim < 2 or array.shape[:2] != input_shape:
            raise ValueError("all aligned arrays must have identical height and width")

    y_indices = nearest_neighbor_indices(input_shape[0], output_height)
    x_indices = nearest_neighbor_indices(input_shape[1], output_width)
    return tuple(
        np.ascontiguousarray(array[y_indices[:, None], x_indices[None, :], ...])
        for array in normalized
    )


def invalidate_xyz_outside_depth_range(
    xyz: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    """Replace non-finite or out-of-range organized XYZ points with NaNs."""
    points = np.asarray(xyz, dtype=np.float32)
    if points.ndim != 3 or points.shape[2] != 3:
        raise ValueError("XYZ data must have shape (H, W, 3)")
    if not np.isfinite(min_depth_m) or min_depth_m < 0.0:
        raise ValueError("min_depth_m must be finite and non-negative")
    if not np.isfinite(max_depth_m) or max_depth_m <= min_depth_m:
        raise ValueError("max_depth_m must be finite and greater than min_depth_m")

    filtered = points.copy()
    z = filtered[:, :, 2]
    valid = (
        np.isfinite(filtered).all(axis=2)
        & (z >= min_depth_m)
        & (z <= max_depth_m)
    )
    filtered[~valid] = np.nan
    return filtered
