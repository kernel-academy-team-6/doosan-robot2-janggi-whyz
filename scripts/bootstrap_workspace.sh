#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${1:-${HOME}/doosan_janggi_ws}"
SOURCE_DIR="${WORKSPACE_DIR}/src"

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy가 없습니다. 먼저 scripts/install_host_ubuntu_24_04.sh를 실행하세요." >&2
    exit 1
  fi
  source /opt/ros/jazzy/setup.bash
fi
set -u

command -v vcs >/dev/null || {
  echo "vcstool이 없습니다: sudo apt install python3-vcstool" >&2
  exit 1
}
command -v colcon >/dev/null || {
  echo "colcon이 없습니다: sudo apt install python3-colcon-common-extensions" >&2
  exit 1
}

mkdir -p "${SOURCE_DIR}"
if find "${SOURCE_DIR}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  echo "${SOURCE_DIR}가 비어 있지 않습니다. 새 워크스페이스를 지정하세요." >&2
  exit 1
fi

vcs import "${SOURCE_DIR}" < "${REPO_DIR}/config/janggi.repos"
git -C "${SOURCE_DIR}/doosan-robot2" apply --check "${REPO_DIR}/patches/doosan-jazzy-flange-serial.patch"
git -C "${SOURCE_DIR}/doosan-robot2" apply "${REPO_DIR}/patches/doosan-jazzy-flange-serial.patch"

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
