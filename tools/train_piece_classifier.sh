#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec env -u PYTHONPATH "$HERE/../janggi/.venv/bin/python" -u "$HERE/train_piece_classifier.py" "$@"
