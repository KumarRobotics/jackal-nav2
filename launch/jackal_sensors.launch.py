#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_ouster = LaunchConfiguration("start_ouster")
    start_zed = LaunchConfiguration("start_zed")
    start_dlio = LaunchConfiguration("start_dlio")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    ouster_container_name = LaunchConfiguration("ouster_container_name")
    camera_model = LaunchConfiguration("camera_model")

    ouster_namespace = PathJoinSubstitution([namespace, "ouster"])

    ouster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ouster_ros"),
                    "launch",
                    "sensor.composite.launch.py",
                ]
            )
        ),
        condition=IfCondition(start_ouster),
        launch_arguments={
            "viz": "false",
            "ouster_ns": ouster_namespace,
        }.items(),
    )

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("zed_wrapper"),
                    "launch",
                    "zed_camera.launch.py",
                ]
            )
        ),
        condition=IfCondition(start_zed),
        launch_arguments={
            "camera_model": camera_model,
            "publish_tf": "false",
            "publish_map_tf": "false",
            "namespace": namespace,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    dlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("direct_lidar_inertial_odometry"),
                    "launch",
                    "dlio.launch.py",
                ]
            )
        ),
        condition=IfCondition(start_dlio),
        launch_arguments={
            "rviz": "false",
            "namespace": namespace,
            "pointcloud_topic": pointcloud_topic,
            "imu_topic": imu_topic,
            "container_name": ouster_container_name,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace shared by the robot sensor nodes",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_ouster", default_value="true"),
            DeclareLaunchArgument("start_zed", default_value="true"),
            DeclareLaunchArgument("start_dlio", default_value="true"),
            DeclareLaunchArgument(
                "pointcloud_topic", default_value="ouster/points"
            ),
            DeclareLaunchArgument("imu_topic", default_value="ouster/imu"),
            DeclareLaunchArgument(
                "ouster_container_name",
                default_value="/ouster/os_container",
                description=(
                    "Existing component container used by DLIO. Override this "
                    "when the Ouster driver is namespaced."
                ),
            ),
            DeclareLaunchArgument("camera_model", default_value="zed2i"),
            ouster,
            zed,
            dlio,
        ]
    )
