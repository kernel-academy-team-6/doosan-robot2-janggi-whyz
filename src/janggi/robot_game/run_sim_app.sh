#!/usr/bin/env bash
# 대국 UI를 카메라 없이 시뮬레이터로 실행. 먼저 가상 로봇(bringup mode:=virtual)을 띄운다.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
exec "$WS/janggi/.venv/bin/python" "$HERE/sim_app.py" "$@"
