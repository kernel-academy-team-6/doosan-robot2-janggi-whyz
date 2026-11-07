#!/usr/bin/env bash
# 장기말 실시간 인식 + 게임 UI. 인자는 app.py로 전달된다.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VENV="$HERE/../.venv"
if [[ ! -x "$VENV/bin/python" ]]; then
  echo "[오류] venv가 없다. 먼저 $HERE/../setup_venv.sh 를 실행한다." >&2
  exit 1
fi
# ROS 환경이 source된 셸에서도 venv 패키지만 쓰도록 PYTHONPATH를 비운다.
exec env -u PYTHONPATH "$VENV/bin/python" "$HERE/app.py" "$@"
