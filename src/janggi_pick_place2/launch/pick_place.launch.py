"""Launch this package only; existing robot bringup remains unchanged."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def launch_node(context):
    overrides = {
        'backend': LaunchConfiguration('backend').perform(context),
        'robot_namespace': LaunchConfiguration('robot_namespace').perform(context),
    }
    length = LaunchConfiguration('tcp_length_at_6mm_mm').perform(context)
    if length:
        overrides['tcp_length_at_6mm_mm'] = float(length)
    return [Node(package='janggi_pick_place2', executable='pick_place_node',
                 name='janggi_pick_place2', output='screen', sigterm_timeout='20',
                 parameters=[LaunchConfiguration('params_file'), overrides])]


def generate_launch_description():
    path = os.path.join(get_package_share_directory('janggi_pick_place2'), 'config', 'pick_place.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=path),
        DeclareLaunchArgument('backend', default_value='mock', choices=['mock', 'virtual', 'real']),
        DeclareLaunchArgument('robot_namespace', default_value='dsr01'),
        DeclareLaunchArgument('tcp_length_at_6mm_mm', default_value='',
                              description='Compatibility override; only fixed 160 mm is allowed. Empty uses YAML.'),
        OpaqueFunction(function=launch_node),
    ])
