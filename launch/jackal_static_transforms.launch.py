#!/usr/bin/env python3

from jackal_nav2.sensor_extrinsics import load_zed_extrinsics

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_transform(parent, child, x, y, z, roll, pitch, yaw, condition=None):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        condition=condition,
        arguments=[
            "--x",
            x,
            "--y",
            y,
            "--z",
            z,
            "--yaw",
            yaw,
            "--pitch",
            pitch,
            "--roll",
            roll,
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


def build_zed_static_transform(context):
    extrinsics_file = LaunchConfiguration("zed_extrinsics_file").perform(context)
    extrinsics = load_zed_extrinsics(extrinsics_file)
    if not extrinsics.calibrated:
        get_logger("jackal_static_transforms").warning(
            "Using provisional, uncalibrated ZED extrinsics from "
            f"{extrinsics_file}. Calibrate this transform before enabling "
            "semantic navigation."
        )

    values = [str(value) for value in extrinsics.transform_values()]
    return [
        static_transform(
            LaunchConfiguration("base_frame"),
            LaunchConfiguration("zed_frame"),
            *values,
        )
    ]


def generate_launch_description():
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    robot_map_frame = LaunchConfiguration("robot_map_frame")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_z = LaunchConfiguration("initial_z")
    initial_yaw = LaunchConfiguration("initial_yaw")
    publish_map_to_odom = LaunchConfiguration("publish_map_to_odom")

    default_zed_extrinsics = PathJoinSubstitution(
        [
            FindPackageShare("jackal_nav2"),
            "config",
            "jackal_sensor_extrinsics.yaml",
        ]
    )

    robot_map_is_empty = PythonExpression(["'", robot_map_frame, "' == ''"])
    direct_map_to_odom = IfCondition(
        AndSubstitution(publish_map_to_odom, robot_map_is_empty)
    )
    use_robot_map_frame = IfCondition(
        AndSubstitution(publish_map_to_odom, NotSubstitution(robot_map_is_empty))
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("robot_map_frame", default_value=""),
            DeclareLaunchArgument("zed_frame", default_value="zed_camera_link"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_map_to_odom", default_value="true"),
            DeclareLaunchArgument(
                "zed_extrinsics_file",
                default_value=default_zed_extrinsics,
                description=(
                    "Packaged or absolute external YAML file containing the "
                    "base-to-ZED transform"
                ),
            ),
            DeclareLaunchArgument(
                "publish_zed_static_tf",
                default_value="true",
                description=(
                    "Publish base_frame to zed_frame from zed_extrinsics_file; "
                    "set false when another TF authority owns this transform"
                ),
            ),
            static_transform(
                map_frame,
                odom_frame,
                initial_x,
                initial_y,
                initial_z,
                "0",
                "0",
                initial_yaw,
                direct_map_to_odom,
            ),
            static_transform(
                map_frame,
                robot_map_frame,
                initial_x,
                initial_y,
                initial_z,
                "0",
                "0",
                initial_yaw,
                use_robot_map_frame,
            ),
            static_transform(
                robot_map_frame,
                odom_frame,
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                use_robot_map_frame,
            ),
            OpaqueFunction(
                function=build_zed_static_transform,
                condition=IfCondition(LaunchConfiguration("publish_zed_static_tf")),
            ),
        ]
    )
