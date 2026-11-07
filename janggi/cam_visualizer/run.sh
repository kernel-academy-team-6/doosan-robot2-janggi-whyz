#!/usr/bin/env bash
# RealSense 3D 시각화 실행. 인자는 cam_visualizer.py로 그대로 전달된다.
set -euo pipefail
cd "$(dirname "$0")"
# ROS 환경이 source된 셸에서도 venv 패키지만 쓰도록 PYTHONPATH를 비운다.
exec env -u PYTHONPATH .venv/bin/python cam_visualizer.py "$@"
