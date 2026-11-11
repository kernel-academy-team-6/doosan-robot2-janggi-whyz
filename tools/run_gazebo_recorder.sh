#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HERE/../install/setup.bash"
set -u
exec /usr/bin/python3 "$HERE/gazebo_recorder.py" "$@"
