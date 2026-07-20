#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def static_transform(parent, child, x, y, z, yaw, condition=None):
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
            "0",
            "--roll",
            "0",
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )


def generate_launch_description():
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    robot_map_frame = LaunchConfiguration("robot_map_frame")
    zed_frame = LaunchConfiguration("zed_frame")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_z = LaunchConfiguration("initial_z")
    initial_yaw = LaunchConfiguration("initial_yaw")

    direct_map_to_odom = IfCondition(
        PythonExpression(["'", robot_map_frame, "' == ''"])
    )
    use_robot_map_frame = UnlessCondition(
        PythonExpression(["'", robot_map_frame, "' == ''"])
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
            DeclareLaunchArgument(
                "publish_zed_static_tf",
                default_value="true",
                description="Publish the existing identity base-to-ZED transform",
            ),
            static_transform(
                map_frame,
                odom_frame,
                initial_x,
                initial_y,
                initial_z,
                initial_yaw,
                direct_map_to_odom,
            ),
            static_transform(
                map_frame,
                robot_map_frame,
                initial_x,
                initial_y,
                initial_z,
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
                use_robot_map_frame,
            ),
            static_transform(
                base_frame,
                zed_frame,
                "0",
                "0",
                "0",
                "0",
                IfCondition(LaunchConfiguration("publish_zed_static_tf")),
            ),
        ]
    )
