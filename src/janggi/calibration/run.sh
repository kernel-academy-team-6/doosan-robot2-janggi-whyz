#!/usr/bin/env bash
# 장기판 4점 카메라-로봇 보정 실행. 인자는 calibrate_board.py로 그대로 전달된다.
# 시스템 Python(rclpy, numpy, cv2) + venv(pyrealsense2) 조합으로 실행한다.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
if [[ ! -x "$HERE/.venv/bin/python" ]]; then
  echo "[오류] venv가 없다. 먼저 $HERE/setup_venv.sh 를 실행한다." >&2
  exit 1
fi
# dsr_msgs2 등 ROS 패키지
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
cd "$HERE"
exec "$HERE/.venv/bin/python" calibrate_board.py "$@"
