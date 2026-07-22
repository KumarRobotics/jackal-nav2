"""Tests for pixel-identical semantic/depth output sampling."""

from jackal_nav2.semantic_terrain import (
    InstanceMask,
    PromptClassConfig,
    SemanticTerrainProcessor,
)
from jackal_nav2.semantic_terrain_sampling import (
    invalidate_xyz_outside_depth_range,
    nearest_neighbor_indices,
    resample_aligned,
)

import numpy as np
import pytest


def test_resample_aligned_uses_identical_source_pixels_for_all_products():
    source_ids = np.arange(24, dtype=np.uint8).reshape(4, 6)
    labels = source_ids
    confidence = source_ids + 30
    xyz = np.stack([source_ids, source_ids + 60, source_ids + 120], axis=2).astype(
        np.float32
    )

    sampled_labels, sampled_confidence, sampled_xyz = resample_aligned(
        (labels, confidence, xyz), output_height=2, output_width=3
    )

    expected_ids = np.array([[0, 2, 4], [12, 14, 16]], dtype=np.uint8)
    np.testing.assert_array_equal(sampled_labels, expected_ids)
    np.testing.assert_array_equal(sampled_confidence, expected_ids + 30)
    np.testing.assert_array_equal(sampled_xyz[:, :, 0], expected_ids)
    np.testing.assert_array_equal(sampled_xyz[:, :, 1], expected_ids + 60)
    np.testing.assert_array_equal(sampled_xyz[:, :, 2], expected_ids + 120)


def test_nearest_neighbor_indices_support_non_integer_scale():
    np.testing.assert_array_equal(nearest_neighbor_indices(5, 3), [0, 1, 3])


def test_depth_range_invalidation_is_inclusive_and_preserves_valid_xyz():
    xyz = np.array(
        [[
            [0.0, 0.0, 0.49],
            [1.0, 2.0, 0.5],
            [3.0, 4.0, 8.0],
            [5.0, 6.0, 8.01],
            [np.nan, 0.0, 1.0],
        ]],
        dtype=np.float32,
    )
    filtered = invalidate_xyz_outside_depth_range(
        xyz, min_depth_m=0.5, max_depth_m=8.0
    )

    assert np.isnan(filtered[0, 0]).all()
    np.testing.assert_array_equal(filtered[0, 1], xyz[0, 1])
    np.testing.assert_array_equal(filtered[0, 2], xyz[0, 2])
    assert np.isnan(filtered[0, 3]).all()
    assert np.isnan(filtered[0, 4]).all()


@pytest.mark.parametrize(
    "minimum,maximum",
    [
        (np.nan, 8.0),
        (0.5, np.nan),
        (np.inf, 8.0),
        (0.5, np.inf),
    ],
)
def test_depth_range_invalidation_rejects_non_finite_bounds(minimum, maximum):
    xyz = np.ones((1, 1, 3), dtype=np.float32)
    with pytest.raises(ValueError, match="finite"):
        invalidate_xyz_outside_depth_range(
            xyz, min_depth_m=minimum, max_depth_m=maximum
        )


class RecordingBackend:
    def __init__(self):
        self.received = None

    def infer(self, bgr_image, prompts):
        self.received = bgr_image.copy()
        return [
            InstanceMask(
                class_index=0,
                score=0.9,
                mask=np.ones(bgr_image.shape[:2], dtype=bool),
            )
        ]

    def close(self):
        pass


def test_processor_preserves_bgr_channel_order_for_injected_backend():
    backend = RecordingBackend()
    config = PromptClassConfig.from_sequences(["road"], [1], ["preferred"])
    processor = SemanticTerrainProcessor(backend, config, score_threshold=0.5)
    bgr = np.array([[[5, 17, 251]]], dtype=np.uint8)

    processor.process(
        bgr,
        np.ones((1, 1), dtype=np.float32),
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )

    np.testing.assert_array_equal(backend.received, bgr)
