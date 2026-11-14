#!/usr/bin/env bash
# Same as janggi/realtime_ui/run.sh, but --classifier may be a hierarchical bundle .json.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
VENV="$WS/janggi/.venv"
robot=0; motion=1; gazebo=0; previous=""
for a in "$@"; do
  [[ "$a" == "--robot" ]] && robot=1
  [[ "$a" == "--no-motion" ]] && motion=0
  if [[ "$a" == "--source=gazebo" || ( "$previous" == "--source" && "$a" == "gazebo" ) ]]; then gazebo=1; fi
  previous="$a"
done
if [[ ( $robot == 1 && $motion == 1 ) || $gazebo == 1 ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
  exec "$VENV/bin/python" "$HERE/app_launcher.py" "$@"
fi
exec env -u PYTHONPATH "$VENV/bin/python" "$HERE/app_launcher.py" "$@"
