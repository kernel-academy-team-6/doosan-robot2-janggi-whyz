#!/usr/bin/env bash
# 시뮬레이터 확인 실행. 시스템 Python(ROS rclpy가 붙은 3.12)으로 돌린다.
# 먼저 다른 터미널에서 가상 로봇을 띄운다:
#   ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual host:=127.0.0.1 port:=12345 name:=dsr01
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
exec /usr/bin/python3 "$HERE/sim_check.py" "$@"
