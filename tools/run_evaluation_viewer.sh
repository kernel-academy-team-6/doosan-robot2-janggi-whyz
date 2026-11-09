#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PYTHON="$HERE/../janggi/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "인식용 venv가 없습니다. 먼저 $HERE/../janggi/setup_venv.sh 를 실행하세요." >&2
  exit 1
fi
exec env -u PYTHONPATH "$PYTHON" "$HERE/evaluation_viewer.py" "$@"
