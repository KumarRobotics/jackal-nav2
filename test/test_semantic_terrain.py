"""Focused tests for costmap-facing semantic terrain products."""

import threading

from jackal_nav2.semantic_terrain import (
    compose_semantic_maps,
    depth_to_xyz,
    InstanceMask,
    LatestFrameMailbox,
    organized_pointcloud2,
    PromptClassConfig,
    render_overlay,
    SemanticTerrainProcessor,
)

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


@pytest.fixture
def terrain_config():
    return PromptClassConfig.from_sequences(
        ["road", "path", "grass"],
        [1, 1, 2],
        ["preferred", "preferred", "caution"],
    )


def test_prompt_config_supports_multiple_prompts_per_stable_class_id(terrain_config):
    assert terrain_config.prompts == ("road", "path", "grass")
    assert terrain_config.prompt_class_ids == (1, 1, 2)
    assert terrain_config.unique_classes == ((1, "preferred"), (2, "caution"))


@pytest.mark.parametrize(
    "prompts,class_ids,class_names,match",
    [
        (["road"], [0], ["preferred"], "outside"),
        (["road", "path"], [1], ["preferred"], "equal lengths"),
        (["road", "road"], [1, 1], ["preferred", "preferred"], "unique"),
        (["road", "path"], [1, 1], ["preferred", "caution"], "maps to both"),
        (["road", "path"], [1, 2], ["preferred", "preferred"], "maps to IDs"),
    ],
)
def test_prompt_config_rejects_ambiguous_mappings(
    prompts, class_ids, class_names, match
):
    with pytest.raises(ValueError, match=match):
        PromptClassConfig.from_sequences(prompts, class_ids, class_names)


def test_compose_semantic_maps_uses_highest_score_and_stable_ties(terrain_config):
    road = np.array([[True, True], [False, False]])
    path = np.array([[True, False], [True, False]])
    grass = np.array([[True, False], [True, True]])
    instances = [
        InstanceMask(class_index=2, score=0.8, mask=grass),
        InstanceMask(class_index=1, score=0.8, mask=path),
        InstanceMask(class_index=0, score=0.6, mask=road),
    ]

    labels, confidence, accepted = compose_semantic_maps(
        (2, 2), instances, terrain_config, score_threshold=0.5
    )

    # Equal-score path/grass overlap is resolved by lower prompt index (path).
    np.testing.assert_array_equal(labels, np.array([[1, 1], [1, 2]], np.uint8))
    np.testing.assert_array_equal(
        confidence, np.array([[204, 153], [204, 204]], np.uint8)
    )
    assert accepted == 3


def test_compose_semantic_maps_filters_low_scores_and_validates_mask_shape(
    terrain_config,
):
    labels, confidence, accepted = compose_semantic_maps(
        (1, 2),
        [InstanceMask(class_index=0, score=0.49, mask=np.ones((1, 2), bool))],
        terrain_config,
        score_threshold=0.5,
    )
    assert not labels.any()
    assert not confidence.any()
    assert accepted == 0

    with pytest.raises(ValueError, match="does not match"):
        compose_semantic_maps(
            (1, 2),
            [InstanceMask(class_index=0, score=0.9, mask=np.ones((2, 2), bool))],
            terrain_config,
            score_threshold=0.5,
        )


def test_depth_to_xyz_handles_metric_float_and_invalid_pixels():
    depth = np.array([[2.0, 0.0], [np.nan, 4.0]], dtype=np.float32)
    xyz = depth_to_xyz(depth, [2.0, 0.0, 0.5, 0.0, 4.0, 0.5, 0.0, 0.0, 1.0])

    np.testing.assert_allclose(xyz[0, 0], [-0.5, -0.25, 2.0])
    np.testing.assert_allclose(xyz[1, 1], [1.0, 0.5, 4.0])
    assert np.isnan(xyz[0, 1]).all()
    assert np.isnan(xyz[1, 0]).all()


def test_depth_to_xyz_scales_integer_depth():
    xyz = depth_to_xyz(
        np.array([[2000]], dtype=np.uint16),
        [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
        integer_depth_scale=0.001,
    )
    np.testing.assert_allclose(xyz[0, 0], [0.0, 0.0, 2.0])


class FakeBackend:
    def __init__(self, instances):
        self.instances = instances
        self.calls = []
        self.closed = False

    def infer(self, bgr_image, prompts):
        self.calls.append((bgr_image.copy(), tuple(prompts)))
        return self.instances

    def close(self):
        self.closed = True


def test_processor_uses_injected_backend_without_sam_or_torch(terrain_config):
    fake = FakeBackend(
        [InstanceMask(2, 0.75, np.array([[True, False], [False, True]]))]
    )
    processor = SemanticTerrainProcessor(
        fake,
        terrain_config,
        score_threshold=0.5,
        integer_depth_scale=0.001,
        output_width=2,
        output_height=2,
        min_depth_m=0.0,
        max_depth_m=2.0,
    )
    bgr = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.full((2, 2), 1000, dtype=np.uint16)

    result = processor.process(
        bgr, depth, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )

    assert fake.calls[0][1] == terrain_config.prompts
    np.testing.assert_array_equal(result.label_mask, [[2, 0], [0, 2]])
    np.testing.assert_array_equal(result.confidence, [[191, 0], [0, 191]])
    assert result.xyz.shape == (2, 2, 3)
    assert result.accepted_instance_count == 1
    assert result.inference_seconds >= 0.0


def test_organized_pointcloud_preserves_pixel_layout_and_nan_values():
    xyz = np.array(
        [
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            [[7.0, 8.0, 9.0], [np.nan, np.nan, np.nan]],
        ],
        dtype=np.float32,
    )
    header = Header(frame_id="camera_optical_frame")
    header.stamp.sec = 42
    cloud = organized_pointcloud2(xyz, header)

    assert isinstance(cloud, PointCloud2)
    assert (cloud.height, cloud.width) == (2, 2)
    assert (cloud.point_step, cloud.row_step) == (12, 24)
    assert [field.name for field in cloud.fields] == ["x", "y", "z"]
    assert cloud.header.frame_id == "camera_optical_frame"
    assert cloud.header.stamp.sec == 42
    assert not cloud.is_dense
    decoded = np.frombuffer(cloud.data, dtype="<f4").reshape(2, 2, 3)
    np.testing.assert_allclose(decoded[:1], xyz[:1])
    assert np.isnan(decoded[1, 1]).all()


def test_overlay_is_deterministic_and_leaves_background_unchanged():
    image = np.full((2, 2, 3), 10, dtype=np.uint8)
    labels = np.array([[0, 1], [2, 1]], dtype=np.uint8)
    first = render_overlay(image, labels, alpha=0.5)
    second = render_overlay(image, labels, alpha=0.5)

    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(first[0, 0], image[0, 0])
    assert not np.array_equal(first[0, 1], image[0, 1])


def test_latest_frame_mailbox_replaces_stale_pending_frame():
    mailbox = LatestFrameMailbox[int]()
    assert mailbox.put(1)
    assert mailbox.put(2)
    assert mailbox.dropped == 1
    assert mailbox.take() == 2

    result = []
    waiter = threading.Thread(target=lambda: result.append(mailbox.take()))
    waiter.start()
    mailbox.close()
    waiter.join(timeout=1.0)
    assert result == [None]
