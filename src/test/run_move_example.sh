#!/usr/bin/env bash
# 장기판 좌표 이동 예제 실행. 시스템 Python(ROS rclpy가 붙은 3.12)으로 돌린다.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
exec /usr/bin/python3 "$HERE/move_board_example.py" "$@"
