#!/usr/bin/env bash
set -euo pipefail

CMAKE_VERSION=3.31.3
ROS_DISTRO=humble
COLCON_WS=/workspace

export DEBIAN_FRONTEND=noninteractive
apt_update() {
  sudo apt-get update -y
}

# Repos and base prep
apt_update
sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo mkdir -p /etc/apt/keyrings
sudo rm -f /etc/apt/keyrings/ros-archive-keyring.asc /etc/apt/keyrings/ros-archive-keyring.gpg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
sudo chmod 644 /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
apt_update
sudo apt-get upgrade -y
sudo apt-get install -y --no-install-recommends \
  locales sudo gosu procps \
  build-essential git wget ninja-build clang clang-format clang-tidy \
  python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool python3-argcomplete \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgpiod-dev libopencv-dev \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-vision-opencv \
  ros-${ROS_DISTRO}-gscam
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

# CMake 3.31.3
cd /tmp
wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.sh
chmod +x cmake-${CMAKE_VERSION}-linux-x86_64.sh
sudo ./cmake-${CMAKE_VERSION}-linux-x86_64.sh --skip-license --prefix=/usr/local
rm cmake-${CMAKE_VERSION}-linux-x86_64.sh

# rosdep setup
sudo rosdep init || true
rosdep update