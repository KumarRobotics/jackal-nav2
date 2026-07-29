#!/usr/bin/env python3

"""Launch the tested subset of Nav2 navigation servers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    nav_cmd_vel_topic = LaunchConfiguration("nav_cmd_vel_topic")
    smoothed_cmd_vel_topic = LaunchConfiguration("smoothed_cmd_vel_topic")

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "waypoint_follower",
    ]

    replaced_params = ReplaceString(
        source_file=params_file,
        replacements={
            "__ODOM_TOPIC__": LaunchConfiguration("odom_topic"),
            "__OBSTACLE_TOPIC__": LaunchConfiguration("obstacle_topic"),
            "__POINTCLOUD_TOPIC__": LaunchConfiguration("pointcloud_topic"),
            "__SPEED_LIMIT_TOPIC__": LaunchConfiguration("speed_limit_topic"),
            "__SMOOTHED_CMD_VEL_TOPIC__": smoothed_cmd_vel_topic,
            "__MAP_FRAME__": LaunchConfiguration("map_frame"),
            "__ODOM_FRAME__": LaunchConfiguration("odom_frame"),
            "__BASE_FRAME__": LaunchConfiguration("base_frame"),
            "__LIDAR_FRAME__": LaunchConfiguration("lidar_frame"),
            "__NAV_TO_POSE_BT_XML__": LaunchConfiguration("nav_to_pose_bt_xml"),
        },
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=replaced_params,
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    controller_remappings = [
        ("cmd_vel", nav_cmd_vel_topic),
    ]
    smoother_remappings = [
        ("cmd_vel", nav_cmd_vel_topic),
        ("cmd_vel_smoothed", smoothed_cmd_vel_topic),
    ]

    node_arguments = ["--ros-args", "--log-level", log_level]
    common_node_options = {
        "namespace": namespace,
        "output": "screen",
        "respawn": use_respawn,
        "respawn_delay": 2.0,
        "parameters": [configured_params],
        "arguments": node_arguments,
    }

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                remappings=controller_remappings,
                **common_node_options,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                **common_node_options,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                **common_node_options,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                remappings=controller_remappings,
                **common_node_options,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                **common_node_options,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                **common_node_options,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                remappings=smoother_remappings,
                **common_node_options,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=namespace,
                output="screen",
                arguments=node_arguments,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    component_options = {
        "namespace": namespace,
        "parameters": [configured_params],
    }
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            LoadComposableNodes(
                target_container=PathJoinSubstitution([namespace, container_name]),
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        remappings=controller_remappings,
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        remappings=controller_remappings,
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        remappings=smoother_remappings,
                        **component_options,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        namespace=namespace,
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "node_names": lifecycle_nodes,
                            }
                        ],
                    ),
                ],
            ),
        ],
    )

    default_params = PathJoinSubstitution(
        [
            FindPackageShare("jackal_nav2"),
            "config",
            "nav2_jackal_stvox_mppi_tuning.yaml",
        ]
    )
    default_bt = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml",
        ]
    )

    arguments = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument("container_name", default_value="nav2_container"),
        DeclareLaunchArgument("use_respawn", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("nav_cmd_vel_topic", default_value="cmd_vel_nav"),
        DeclareLaunchArgument(
            "smoothed_cmd_vel_topic", default_value="cmd_vel_smoothed"
        ),
        DeclareLaunchArgument("odom_topic", default_value="dlio/odom_node/odom"),
        DeclareLaunchArgument(
            "obstacle_topic", default_value="/groundgrid/obstacle_cloud"
        ),
        DeclareLaunchArgument("pointcloud_topic", default_value="/ouster/points"),
        DeclareLaunchArgument("speed_limit_topic", default_value="speed_limit"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_frame", default_value="os_lidar"),
        DeclareLaunchArgument("nav_to_pose_bt_xml", default_value=default_bt),
    ]

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            *arguments,
            load_nodes,
            load_composable_nodes,
        ]
    )
