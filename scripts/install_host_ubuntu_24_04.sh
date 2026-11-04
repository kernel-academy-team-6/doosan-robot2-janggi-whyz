#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  echo "이 스크립트는 일반 사용자로 실행하고 sudo 암호를 입력하세요." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "24.04" ]]; then
  echo "지원 환경은 Ubuntu 24.04입니다. 현재: ${PRETTY_NAME}" >&2
  exit 1
fi

sudo apt update
sudo apt install -y locales software-properties-common curl ca-certificates git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe

if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION="$({ curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest || true; } | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n 1)"
  if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
    echo "ros2-apt-source 최신 버전을 확인하지 못했습니다." >&2
    exit 1
  fi
  ROS_APT_SOURCE_DEB="/tmp/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.deb"
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" -o "${ROS_APT_SOURCE_DEB}"
  sudo dpkg -i "${ROS_APT_SOURCE_DEB}"
fi

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  netcat-openbsd

if ! command -v docker >/dev/null; then
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  ARCHITECTURE="$(dpkg --print-architecture)"
  sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: ${ARCHITECTURE}
Signed-By: /etc/apt/keyrings/docker.asc
EOF
  sudo apt update
  sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

echo
echo "호스트 설치가 끝났습니다."
echo "Docker 그룹 가입은 자동으로 하지 않았습니다. 다음 중 하나를 사용하세요:"
echo "  sudo docker ..."
echo "  또는 위험을 이해한 뒤: sudo usermod -aG docker \"${USER}\""
