#!/usr/bin/env bash
set -euo pipefail

CMAKE_VERSION=3.31.3
ROS_DISTRO=humble
COLCON_WS=/workspace

# Helper to keep noninteractive apt runs tidy
export DEBIAN_FRONTEND=noninteractive
apt_update() {
  sudo apt-get update -y
}

# Base prep
apt_update
sudo apt-get upgrade -y
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release locales sudo gosu procps \
  build-essential git wget ninja-build clang clang-format clang-tidy \
  python3-pip python3-rosdep python3-colcon-common-extensions python3-vcstool python3-argcomplete \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libgpiod-dev libopencv-dev

# ROS 2 Humble
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros-archive-keyring.asc >/dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# Install ROS 2 desktop
apt_update
sudo apt-get install -y --no-install-recommends \
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