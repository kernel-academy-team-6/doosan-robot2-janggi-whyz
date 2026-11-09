"""가상 로봇(에뮬레이터) + Gazebo. 장기 대국 시뮬레이션 보기용.

    ros2 launch src/janggi/robot_game/sim_gazebo.launch.py          # (워크스페이스 루트에서)

- dsr01: e0509_gripper_description/bringup.launch.py mode:=virtual (에뮬레이터, 실제로 제어하는 로봇)
- Gazebo: 빈 world만 띄운다. 로봇·장기판·말은 sim_check.py / sim_app.py --gazebo가 gz_scene.py로 만들고,
  로봇은 /dsr01/joint_states로 순기구학을 풀어 링크 위치를 그대로 옮긴다 (보기 전용, 물리 없음).

e0509_gripper_description의 bringup_real_gazebo.launch.py(gz_ros2_control 트윈)는 지금 빌드에서
실행 파일 이름(.py)과 mimic 관절 설정 때문에 뜨지 않아서 쓰지 않는다.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("e0509_gripper_description")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
        launch_arguments={"mode": "virtual", "host": "127.0.0.1", "port": "12345", "name": "dsr01",
                          "rviz": LaunchConfiguration("rviz")}.items(),
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": "-r -v 2 empty.sdf"}.items(),
    )
    return LaunchDescription([DeclareLaunchArgument("rviz", default_value="false"), robot, gazebo])
