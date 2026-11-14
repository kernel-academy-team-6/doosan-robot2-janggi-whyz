"""Virtual Doosan driver + physically simulated Gazebo robot and pieces.

The emulator supplies joint targets. Gazebo joint feedback controllers apply bounded torques;
contact, gravity and friction determine the pieces' actual motion. No pose mirror.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    from importlib.util import spec_from_file_location, module_from_spec
    spec = spec_from_file_location("janggi_simulation_calibration", os.path.join(here, "simulation_calibration.py"))
    calibration_module = module_from_spec(spec); spec.loader.exec_module(calibration_module)
    simulation_calibration = calibration_module.ensure_simulation_calibration()
    pkg = get_package_share_directory("e0509_gripper_description")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
        launch_arguments={"mode": "virtual", "host": "127.0.0.1", "port": "12345", "name": "dsr01",
                          "rviz": LaunchConfiguration("rviz"),
                          "gripper_mount_offset_mm": LaunchConfiguration("gripper_mount_offset_mm")}.items(),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r -v 2 --physics-engine gz-physics-bullet-featherstone-plugin ", LaunchConfiguration("world")]}.items(),
    )
    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="janggi_camera_bridge",
        arguments=[
            "/janggi_sim/d455f/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/janggi_sim/d455f/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[("/janggi_sim/d455f/image", "/janggi_sim/d455f/color/image_raw"),
                    ("/janggi_sim/d455f/camera_info", "/janggi_sim/d455f/color/camera_info")],
        condition=IfCondition(LaunchConfiguration("camera_bridge")), output="screen")
    with open(simulation_calibration) as stream:
        transform = yaml.safe_load(stream)["T_base_camera"]
    with open(os.path.join(here, "config", "scene.yaml")) as stream:
        optical_frame = yaml.safe_load(stream)["camera"]["optical_frame"]
    tf_args = []
    for flag, value in zip(("--x", "--y", "--z", "--qx", "--qy", "--qz", "--qw"),
                           transform["translation"] + transform["rotation_xyzw"]):
        tf_args.extend([flag, str(value)])
    camera_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="janggi_camera_tf",
        arguments=tf_args + ["--frame-id", "base_link", "--child-frame-id", optical_frame],
        output="screen")
    scene = ExecuteProcess(
        cmd=["/usr/bin/python3", os.path.join(here, "gazebo_scene.py"), "--wait-for-gui", "--ns", LaunchConfiguration("name")],
        condition=IfCondition(LaunchConfiguration("scene")), output="screen")
    # PRIME render offload: Gazebo server (camera) and GUI render on the NVIDIA GPU.
    # Measured 2026-11-14: same RTF as the Intel iGPU (physics is the bottleneck).
    use_nvidia = IfCondition(PythonExpression(["'", LaunchConfiguration("gpu"), "' == 'nvidia'"]))
    nvidia_env = [SetEnvironmentVariable(name, value, condition=use_nvidia) for name, value in (
        ("__NV_PRIME_RENDER_OFFLOAD", "1"), ("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        ("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"))]
    return LaunchDescription([
        DeclareLaunchArgument("gpu", default_value="nvidia", description="nvidia: RTX via PRIME offload | default: system OpenGL (Intel iGPU)"),
        *nvidia_env,
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gripper_mount_offset_mm", default_value="13.766",
                              description="Gripper mounting gap along flange link_6 local +Z [mm]"),
        DeclareLaunchArgument("world", default_value=os.path.join(here, "worlds", "janggi_camera.sdf")),
        DeclareLaunchArgument("camera_bridge", default_value="false", description="Enable ROS image bridge only for external ROS consumers; UI defaults to native Gazebo Transport"),
        DeclareLaunchArgument("scene", default_value="true", description="Create board/camera, dynamic pieces and force-driven robot"),
        robot, gazebo, camera_bridge, camera_tf, scene])
