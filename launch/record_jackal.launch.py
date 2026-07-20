#!/usr/bin/env python3

"""Record explicit Jackal autonomy topic profiles with rosbag2."""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def namespaced_topic(namespace: str, topic: str) -> str:
    if topic.startswith("/"):
        return topic
    clean_namespace = namespace.strip("/")
    if clean_namespace:
        return f"/{clean_namespace}/{topic.lstrip('/')}"
    return f"/{topic.lstrip('/')}"


def build_recorder(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    output_root = Path(
        LaunchConfiguration("output_root").perform(context)
    ).expanduser()
    output_bag = LaunchConfiguration("output_bag").perform(context).strip()
    extra_topics = LaunchConfiguration("extra_topics").perform(context)
    zed_prefix = LaunchConfiguration("zed_topic_prefix").perform(context).strip("/")

    navigation_topics = [
        "dlio/odom_node/odom",
        "dlio/odom_node/path",
        "groundgrid/obstacle_cloud",
        "global_costmap/costmap",
        "local_costmap/costmap",
        "plan",
        "platform/cmd_vel",
        "cmd_vel_smoothed",
        "joy",
        "/tf",
        "/tf_static",
    ]
    lidar_topics = [
        "ouster/points",
        "ouster/imu",
        "dlio/odom_node/pointcloud/deskewed",
    ]
    semantic_topics = [
        f"{zed_prefix}/left/image_rect_color/compressed",
        f"{zed_prefix}/left/camera_info",
        f"{zed_prefix}/depth/depth_registered",
        f"{zed_prefix}/depth/camera_info",
    ]

    profiles = {
        "navigation": navigation_topics,
        "lidar": navigation_topics + lidar_topics,
        "semantic": navigation_topics + semantic_topics,
        "full": navigation_topics + lidar_topics + semantic_topics,
    }
    if profile not in profiles:
        raise RuntimeError(
            f"Unknown recording profile {profile!r}; "
            f"choose one of {', '.join(profiles)}"
        )

    topics = [namespaced_topic(namespace, topic) for topic in profiles[profile]]
    topics.extend(
        namespaced_topic(namespace, topic.strip())
        for topic in extra_topics.split(",")
        if topic.strip()
    )
    topics = list(dict.fromkeys(topics))

    if output_bag:
        bag_path = Path(output_bag).expanduser()
        bag_path.parent.mkdir(parents=True, exist_ok=True)
    else:
        output_root.mkdir(parents=True, exist_ok=True)
        bag_path = output_root / (
            "jackal-autonomy-" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        )

    command = ["ros2", "bag", "record", *topics, "-o", str(bag_path)]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument(
                "profile",
                default_value="navigation",
                description="One of navigation, lidar, semantic, or full",
            ),
            DeclareLaunchArgument("output_root", default_value="~/data/bags"),
            DeclareLaunchArgument(
                "output_bag",
                default_value="",
                description="Explicit bag path; overrides timestamped output_root",
            ),
            DeclareLaunchArgument(
                "extra_topics",
                default_value="",
                description="Comma-separated additional topics",
            ),
            DeclareLaunchArgument(
                "zed_topic_prefix",
                default_value="zed/zed_node",
                description="ZED topic prefix below the robot namespace",
            ),
            OpaqueFunction(function=build_recorder),
        ]
    )
