"""Publish only the configured camera mounting edge; no sensors or motion nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from jackal_nav2.camera_mount import static_arguments


def configure(context):
    def arg(name):return LaunchConfiguration(name).perform(context)
    return [Node(package='tf2_ros',executable='static_transform_publisher',name='zed_mount_tf',output='screen',
        arguments=static_arguments(arg('zed_mount_config'),arg('parent_frame'),arg('child_frame')),
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():
    default=Path(get_package_share_directory('jackal_nav2'))/'config/zed_mount.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('zed_mount_config',default_value=str(default)),
        DeclareLaunchArgument('parent_frame',default_value='',description='Optional override of the YAML parent frame'),
        DeclareLaunchArgument('child_frame',default_value='',description='Optional override of the YAML camera root frame'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        OpaqueFunction(function=configure)])
