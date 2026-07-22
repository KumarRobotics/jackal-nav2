from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_semantic_overlay():
    path = PACKAGE_ROOT / "config" / "nav2_semantic_terrain_overlay.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def semantic_layer(config, costmap_name):
    return config[costmap_name][costmap_name]["ros__parameters"][
        "semantic_layer"
    ]


def test_both_costmaps_fuse_geometry_then_semantics_then_inflation():
    config = load_semantic_overlay()
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        assert params["plugins"] == [
            "voxel_layer",
            "semantic_layer",
            "inflation_layer",
        ]
        assert params["semantic_layer"]["combination_method"] == 1


def test_semantic_policy_is_soft_cost_only_and_matches_published_labels():
    config = load_semantic_overlay()
    expected_classes = {
        "preferred_surface",
        "caution_surface",
        "high_risk_surface",
        "water_or_dropoff",
    }
    for costmap_name in ("local_costmap", "global_costmap"):
        source = semantic_layer(config, costmap_name)["zed_semantics"]
        assert set(source["class_types"]) == expected_classes
        assert source["segmentation_topic"] == "semantic_terrain/label_mask"
        assert source["confidence_topic"] == "semantic_terrain/confidence"
        assert source["pointcloud_topic"] == "semantic_terrain/points"
        assert max(source[name]["max_cost"] for name in expected_classes) == 252
        assert all(source[name]["max_cost"] < 254 for name in expected_classes)


def test_semantic_profile_applies_initial_physical_speed_cap():
    config = load_semantic_overlay()
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]
    assert controller["vx_max"] == 0.3
    assert controller["vx_min"] == -0.3
    assert smoother["max_velocity"][0] == 0.3
    assert smoother["min_velocity"][0] == -0.3
