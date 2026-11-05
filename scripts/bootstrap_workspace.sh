#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SOURCE_DIR="${WORKSPACE_DIR}/src"
PATCH_FILE="${WORKSPACE_DIR}/patches/doosan-jazzy-flange-serial.patch"

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy가 없습니다. 먼저 scripts/install_host_ubuntu_24_04.sh를 실행하세요." >&2
    exit 1
  fi
  source /opt/ros/jazzy/setup.bash
fi
set -u

command -v colcon >/dev/null || {
  echo "colcon이 없습니다: sudo apt install python3-colcon-common-extensions" >&2
  exit 1
}

git -C "${WORKSPACE_DIR}" submodule update --init --recursive

if git -C "${SOURCE_DIR}/doosan-robot2" apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Flange Serial 패치가 이미 적용되어 있습니다."
else
  git -C "${SOURCE_DIR}/doosan-robot2" apply --check "${PATCH_FILE}"
  git -C "${SOURCE_DIR}/doosan-robot2" apply "${PATCH_FILE}"
fi

rosdep update
rosdep install --from-paths "${SOURCE_DIR}" --ignore-src -r -y --rosdistro jazzy

cd "${WORKSPACE_DIR}"
colcon build \
  --symlink-install \
  --packages-up-to e0509_gripper_description dsr_bringup2 dsr_controller2 \
  --cmake-args -DDRCF_VER=3

echo
echo "빌드 완료"
echo "새 터미널에서 다음을 실행하세요:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
