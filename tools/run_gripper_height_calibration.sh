#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
exec /usr/bin/python3 "$HERE/gripper_height_tool.py" "$@"
