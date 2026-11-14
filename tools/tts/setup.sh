#!/usr/bin/env bash
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
venv="$ws/tools/.venv-supertonic"
# ROS PYTHONPATH must not leak into the separate generator's dependency resolver.
unset PYTHONPATH PYTHONHOME
if [[ ! -x "$venv/bin/python" ]]; then
    python3 -m venv --without-pip "$venv"
fi
if ! "$venv/bin/python" -m pip --version >/dev/null 2>&1; then
    if [[ -x "$ws/janggi/.venv/bin/python" ]]; then
        "$ws/janggi/.venv/bin/python" -m pip --python "$venv/bin/python" install --no-cache-dir pip
    else
        echo 'pip bootstrap unavailable: install python3-venv, then recreate tools/.venv-supertonic.' >&2
        exit 1
    fi
fi
"$venv/bin/python" -m pip install --no-cache-dir -r "$ws/tools/tts/requirements.lock.txt"
"$venv/bin/python" -m pip check
"$venv/bin/python" "$ws/tools/tts/generate.py" "$@"
