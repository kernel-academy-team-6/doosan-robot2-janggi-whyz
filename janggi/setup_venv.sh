#!/usr/bin/env bash
# piece_detection / realtime_ui 공용 venv. torch(CUDA) + ultralytics + pyrealsense2.
# ROS가 쓰는 시스템 numpy와 섞이지 않도록 system-site-packages 없이 만든다.
set -euo pipefail
cd "$(dirname "$0")"
unset PYTHONPATH
/usr/bin/python3 -m venv --without-pip .venv
curl -sSL https://bootstrap.pypa.io/get-pip.py | .venv/bin/python - -q
.venv/bin/pip install -q -r requirements.txt
.venv/bin/python -c "import torch, ultralytics, pyrealsense2, cv2, tkinter, PIL.ImageTk; print('ok torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'ultralytics', ultralytics.__version__)"
