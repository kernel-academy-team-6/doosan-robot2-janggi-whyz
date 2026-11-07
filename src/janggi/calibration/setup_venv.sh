#!/usr/bin/env bash
# 시스템 site-packages(ROS rclpy, python3-numpy, python3-opencv)를 그대로 쓰고
# pyrealsense2만 venv에 추가한다. numpy 2.x를 섞으면 ROS 메시지 모듈이 깨지므로 numpy는 설치하지 않는다.
set -euo pipefail
cd "$(dirname "$0")"
/usr/bin/python3 -m venv --system-site-packages --without-pip .venv
curl -sSL https://bootstrap.pypa.io/get-pip.py | .venv/bin/python - -q
.venv/bin/pip install -q -r requirements.txt
.venv/bin/python -c "import pyrealsense2, numpy, cv2, yaml, tkinter, PIL.ImageTk; print('ok: numpy', numpy.__version__, 'cv2', cv2.__version__)"
