#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${1:-${HOME}/doosan_janggi_ws}"
source /opt/ros/jazzy/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
elif [[ -f "${WORKSPACE_DIR}/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/setup.bash"
else
  echo "설치 setup 파일을 찾지 못했습니다: ${WORKSPACE_DIR}" >&2
  exit 1
fi
set -u

required_packages=(
  dsr_bringup2
  dsr_controller2
  dsr_hardware2
  dsr_msgs2
  e0509_gripper_description
  rh_p12_rn_a_description
)

for package_name in "${required_packages[@]}"; do
  prefix="$(ros2 pkg prefix "${package_name}")"
  printf 'OK  %-30s %s\n' "${package_name}" "${prefix}"
done

required_interfaces=(
  dsr_msgs2/srv/FlangeSerialOpen
  dsr_msgs2/srv/FlangeSerialClose
  dsr_msgs2/srv/FlangeSerialWrite
  dsr_msgs2/srv/FlangeSerialRead
)

for interface_name in "${required_interfaces[@]}"; do
  ros2 interface show "${interface_name}" >/dev/null
  printf 'OK  %s\n' "${interface_name}"
done

echo "정적 설치 검증을 통과했습니다. 실제 로봇 검증은 docs/RUNBOOK.md를 따르세요."
