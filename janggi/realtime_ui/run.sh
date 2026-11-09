#!/usr/bin/env bash
# 장기말 실시간 인식 + 게임 UI. 인자는 app.py로 전달된다.
#   ./run.sh --robot     사람 ↔ 로봇 대국 (ROS 환경을 불러와 venv python에서 rclpy를 쓴다)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
VENV="$HERE/../.venv"
if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[오류] venv가 없다. 먼저 $HERE/../setup_venv.sh 를 실행한다." >&2
  exit 1
fi
robot=0; motion=1
for a in "$@"; do
  [[ "$a" == "--robot" ]] && robot=1
  [[ "$a" == "--no-motion" ]] && motion=0
done
if [[ $robot == 1 && $motion == 1 ]]; then
  # 로봇 대국: ROS(rclpy, dsr_msgs2)를 쓰므로 ROS 환경을 불러온다. venv 패키지(numpy, torch 등)가 우선한다.
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$WS/install/setup.bash"
  set -u
  exec "$VENV/bin/python" "$HERE/app.py" "$@"
fi
# ROS 환경이 source된 셸에서도 venv 패키지만 쓰도록 PYTHONPATH를 비운다.
exec env -u PYTHONPATH "$VENV/bin/python" "$HERE/app.py" "$@"
