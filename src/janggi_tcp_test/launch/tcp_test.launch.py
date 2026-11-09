from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def start(context):
    overrides = {'backend':LaunchConfiguration('backend').perform(context),
                 'robot_namespace':LaunchConfiguration('robot_namespace').perform(context)}
    length = LaunchConfiguration('tcp_length_at_6mm_mm').perform(context)
    if length:
        overrides['tcp_length_at_6mm_mm'] = float(length)
    return [Node(package='janggi_tcp_test',executable='tcp_test_node',name='janggi_tcp_test',
                 output='screen',sigterm_timeout='30',parameters=[LaunchConfiguration('params_file'),overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(
            get_package_share_directory('janggi_tcp_test'),'config','tcp_test.yaml')),
        DeclareLaunchArgument('backend',default_value='mock',choices=['mock','virtual','real']),
        DeclareLaunchArgument('robot_namespace',default_value='dsr01'),
        DeclareLaunchArgument('tcp_length_at_6mm_mm',default_value=''),
        OpaqueFunction(function=start),
    ])
