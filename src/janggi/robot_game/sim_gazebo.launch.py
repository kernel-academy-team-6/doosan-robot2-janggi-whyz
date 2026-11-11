"""가상 로봇(에뮬레이터) + Gazebo. 장기 대국 시뮬레이션 보기용.

    ros2 launch src/janggi/robot_game/sim_gazebo.launch.py          # (워크스페이스 루트에서)

- dsr01: e0509_gripper_description/bringup.launch.py mode:=virtual (에뮬레이터, 실제로 제어하는 로봇)
- Gazebo: Sensors 플러그인이 있는 world를 띄운다. 로봇·장기판·말·D455f는 sim_check.py / sim_app.py --gazebo가 만들고,
  로봇은 /dsr01/joint_states로 순기구학을 풀어 링크 위치를 그대로 옮긴다 (보기 전용, 물리 없음).

e0509_gripper_description의 bringup_real_gazebo.launch.py(gz_ros2_control 트윈)는 지금 빌드에서
실행 파일 이름(.py)과 mimic 관절 설정 때문에 뜨지 않아서 쓰지 않는다.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    pkg = get_package_share_directory("e0509_gripper_description")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
        launch_arguments={"mode": "virtual", "host": "127.0.0.1", "port": "12345", "name": "dsr01",
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": ["-r -v 2 ", LaunchConfiguration("world")]}.items(),
    )
    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="janggi_camera_bridge",
        arguments=[
            "/janggi_sim/d455f/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/janggi_sim/d455f/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/janggi_sim/d455f/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings=[("/janggi_sim/d455f/image", "/janggi_sim/d455f/color/image_raw"),
                    ("/janggi_sim/d455f/depth_image", "/janggi_sim/d455f/aligned_depth_to_color/image_raw"),
                    ("/janggi_sim/d455f/camera_info", "/janggi_sim/d455f/color/camera_info")],
        condition=IfCondition(LaunchConfiguration("camera_bridge")), output="screen")
    with open(os.path.join(here, "..", "calibration", "config", "calibration.yaml")) as stream:
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
        condition=IfCondition(LaunchConfiguration("camera_bridge")), output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("world", default_value=os.path.join(here, "worlds", "janggi_camera.sdf")),
        DeclareLaunchArgument("camera_bridge", default_value="true"),
        robot, gazebo, camera_bridge, camera_tf])
