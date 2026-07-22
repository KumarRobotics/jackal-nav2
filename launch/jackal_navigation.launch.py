#!/usr/bin/env python3

"""Launch GroundGrid, static transforms, Nav2, and the Jackal command bridge."""

from jackal_nav2.sensor_extrinsics import zed_extrinsics_are_calibrated

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def validate_semantic_extrinsics(context):
    semantic_navigation = IfCondition(
        LaunchConfiguration("semantic_navigation")
    ).evaluate(context)
    start_nav2 = IfCondition(
        LaunchConfiguration("start_nav2")
    ).evaluate(context)
    require_calibration = IfCondition(
        LaunchConfiguration("require_calibrated_extrinsics")
    ).evaluate(context)
    if not (semantic_navigation and start_nav2 and require_calibration):
        return []

    extrinsics_file = LaunchConfiguration("zed_extrinsics_file").perform(context)
    if not zed_extrinsics_are_calibrated(extrinsics_file):
        raise RuntimeError(
            "Semantic navigation requires calibrated ZED extrinsics. "
            f"Update {extrinsics_file} and set zed.calibrated to true, or "
            "set require_calibrated_extrinsics:=false only for stationary/offline "
            "testing where robot motion is independently disabled."
        )
    return []


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    smoothed_cmd_vel_topic = LaunchConfiguration("smoothed_cmd_vel_topic")
    semantic_terrain_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("semantic_terrain_params_file"),
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    semantic_terrain = Node(
        package="jackal_nav2",
        executable="semantic_terrain",
        name="semantic_terrain",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("semantic_navigation")),
        parameters=[
            semantic_terrain_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("jackal_nav2"),
                    "launch",
                    "jackal_static_transforms.launch.py",
                ]
            )
        ),
        launch_arguments={
            "map_frame": map_frame,
            "publish_map_to_odom": LaunchConfiguration(
                "publish_map_to_odom"
            ),
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "robot_map_frame": LaunchConfiguration("robot_map_frame"),
            "zed_frame": LaunchConfiguration("zed_frame"),
            "zed_extrinsics_file": LaunchConfiguration(
                "zed_extrinsics_file"
            ),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_z": LaunchConfiguration("initial_z"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
            "publish_zed_static_tf": LaunchConfiguration(
                "publish_zed_static_tf"
            ),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    groundgrid = Node(
        package="groundgrid",
        executable="groundgrid_node",
        name="groundgrid_node",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_groundgrid")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "z_threshold": LaunchConfiguration("ground_grid_z_threshold"),
                "odom_topic": odom_topic,
                "pointcloud_topic": pointcloud_topic,
                "grid_map_topic": "groundgrid/grid_map",
                "segmented_cloud_topic": "groundgrid/segmented_cloud",
                "obstacle_cloud_topic": obstacle_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "lidar_frame": lidar_frame,
            }
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("jackal_nav2"),
                    "launch",
                    "nav2_servers.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_nav2")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "semantic_navigation": LaunchConfiguration("semantic_navigation"),
            "semantic_params_file": LaunchConfiguration(
                "semantic_params_file"
            ),
            "semantic_nav_to_pose_bt_xml": LaunchConfiguration(
                "semantic_nav_to_pose_bt_xml"
            ),
            "autostart": LaunchConfiguration("autostart"),
            "use_composition": LaunchConfiguration("use_composition"),
            "container_name": LaunchConfiguration("nav2_container_name"),
            "use_respawn": LaunchConfiguration("use_respawn"),
            "log_level": LaunchConfiguration("log_level"),
            "nav_cmd_vel_topic": LaunchConfiguration("nav_cmd_vel_topic"),
            "smoothed_cmd_vel_topic": smoothed_cmd_vel_topic,
            "odom_topic": odom_topic,
            "obstacle_topic": obstacle_topic,
            "pointcloud_topic": pointcloud_topic,
            "speed_limit_topic": LaunchConfiguration("speed_limit_topic"),
            "map_frame": map_frame,
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "lidar_frame": lidar_frame,
        }.items(),
    )

    joy_bridge = Node(
        package="jackal_nav2",
        executable="cmd_vel_to_joy",
        name="cmd_vel_to_joy",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_joy_bridge")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": smoothed_cmd_vel_topic,
                "joy_input_topic": LaunchConfiguration("joy_input_topic"),
                "joy_output_topic": LaunchConfiguration("joy_output_topic"),
                "linear_speed_at_full_axis": LaunchConfiguration(
                    "linear_speed_at_full_axis"
                ),
                "angular_speed_at_full_axis": LaunchConfiguration(
                    "angular_speed_at_full_axis"
                ),
            }
        ],
    )

    default_params = PathJoinSubstitution(
        [
            FindPackageShare("jackal_nav2"),
            "config",
            "nav2_jackal_stvox_mppi_tuning.yaml",
        ]
    )
    default_semantic_params = PathJoinSubstitution(
        [
            FindPackageShare("jackal_nav2"),
            "config",
            "nav2_semantic_terrain_overlay.yaml",
        ]
    )
    default_semantic_terrain_params = PathJoinSubstitution(
        [
            FindPackageShare("jackal_nav2"),
            "config",
            "semantic_terrain.yaml",
        ]
    )
    default_semantic_bt = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_to_pose_w_replanning_and_recovery.xml",
        ]
    )
    arguments = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("semantic_navigation", default_value="false"),
        DeclareLaunchArgument(
            "semantic_params_file",
            default_value=default_semantic_params,
        ),
        DeclareLaunchArgument(
            "semantic_terrain_params_file",
            default_value=default_semantic_terrain_params,
        ),
        DeclareLaunchArgument(
            "semantic_nav_to_pose_bt_xml",
            default_value=default_semantic_bt,
        ),
        DeclareLaunchArgument("require_calibrated_extrinsics", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument("nav2_container_name", default_value="nav2_container"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_groundgrid", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="true"),
        DeclareLaunchArgument("start_joy_bridge", default_value="true"),
        DeclareLaunchArgument("publish_map_to_odom", default_value="true"),
        DeclareLaunchArgument("pointcloud_topic", default_value="ouster/points"),
        DeclareLaunchArgument("odom_topic", default_value="dlio/odom_node/odom"),
        DeclareLaunchArgument(
            "obstacle_topic", default_value="groundgrid/obstacle_cloud"
        ),
        DeclareLaunchArgument("nav_cmd_vel_topic", default_value="platform/cmd_vel"),
        DeclareLaunchArgument(
            "smoothed_cmd_vel_topic", default_value="cmd_vel_smoothed"
        ),
        DeclareLaunchArgument("speed_limit_topic", default_value="speed_limit"),
        DeclareLaunchArgument("joy_input_topic", default_value="joy"),
        DeclareLaunchArgument("joy_output_topic", default_value="joy"),
        DeclareLaunchArgument("linear_speed_at_full_axis", default_value="2.0"),
        DeclareLaunchArgument("angular_speed_at_full_axis", default_value="0.75"),
        DeclareLaunchArgument("ground_grid_z_threshold", default_value="1.5"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="os_lidar"),
        DeclareLaunchArgument("zed_frame", default_value="zed_camera_link"),
        DeclareLaunchArgument(
            "zed_extrinsics_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("jackal_nav2"),
                    "config",
                    "jackal_sensor_extrinsics.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument("robot_map_frame", default_value=""),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("publish_zed_static_tf", default_value="true"),
    ]

    return LaunchDescription(
        [
            *arguments,
            OpaqueFunction(function=validate_semantic_extrinsics),
            groundgrid,
            static_transforms,
            semantic_terrain,
            nav2,
            joy_bridge,
        ]
    )
