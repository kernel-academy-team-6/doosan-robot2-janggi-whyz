#!/usr/bin/env bash
set -euo pipefail

IMAGE="doosanrobot/dsr_emulator:3.0.1"

if docker info >/dev/null 2>&1; then
  docker pull "${IMAGE}"
else
  echo "현재 사용자는 Docker daemon에 접근할 수 없어 sudo를 사용합니다."
  sudo docker pull "${IMAGE}"
fi
