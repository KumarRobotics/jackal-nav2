#!/usr/bin/env python3

"""Launch GroundGrid, static transforms, Nav2, and Jackal support nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        condition=IfCondition(LaunchConfiguration("publish_map_to_odom")),
        launch_arguments={
            "map_frame": map_frame,
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "robot_map_frame": LaunchConfiguration("robot_map_frame"),
            "zed_frame": LaunchConfiguration("zed_frame"),
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

    motion_analysis = Node(
        package="jackal_nav2",
        executable="motion_stats",
        name="motion_stats",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_motion_analysis")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "nav_cmd_vel_topic": LaunchConfiguration("nav_cmd_vel_topic"),
                "smoothed_cmd_vel_topic": smoothed_cmd_vel_topic,
                "odom_topic": odom_topic,
                "output_directory": LaunchConfiguration(
                    "motion_analysis_output_directory"
                ),
                "report_interval": LaunchConfiguration(
                    "motion_analysis_report_interval"
                ),
                "acceleration_filter_alpha": LaunchConfiguration(
                    "motion_analysis_acceleration_filter_alpha"
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
    arguments = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument("nav2_container_name", default_value="nav2_container"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("start_groundgrid", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="true"),
        DeclareLaunchArgument("start_joy_bridge", default_value="true"),
        DeclareLaunchArgument("start_motion_analysis", default_value="false"),
        DeclareLaunchArgument(
            "motion_analysis_output_directory", default_value=""
        ),
        DeclareLaunchArgument(
            "motion_analysis_report_interval", default_value="30.0"
        ),
        DeclareLaunchArgument(
            "motion_analysis_acceleration_filter_alpha", default_value="0.25"
        ),
        DeclareLaunchArgument("publish_map_to_odom", default_value="true"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/ouster/points"),
        DeclareLaunchArgument("odom_topic", default_value="dlio/odom_node/odom"),
        DeclareLaunchArgument(
            "obstacle_topic", default_value="/groundgrid/obstacle_cloud"
        ),
        DeclareLaunchArgument("nav_cmd_vel_topic", default_value="cmd_vel_nav"),
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
            groundgrid,
            static_transforms,
            nav2,
            joy_bridge,
            motion_analysis,
        ]
    )
