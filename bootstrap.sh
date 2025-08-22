#!/usr/bin/env bash
set -euo pipefail

WS=$HOME/logistics-robot-control-practice  # 워크스페이스 = 지금 repo
cd "$WS"

echo "[1/6] 도구 설치"
sudo apt-get update
sudo apt-get install -y \
  git git-lfs curl \
  python3-vcstool python3-colcon-common-extensions python3-rosdep \
  gazebo libgazebo-dev
git lfs install

echo "[2/6] rosdep 초기화/업데이트"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "[3/6] ROS 2 Humble 확인"
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS 2 Humble 필요"; exit 1; }
source /opt/ros/humble/setup.bash

echo "[4/6] 의존성 설치"
rosdep install --from-paths src --rosdistro humble -y --ignore-src

echo "[5/6] 빌드"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[6/6] 완료"
echo "   source \"$WS/install/setup.bash\""
echo "   git -C \"$WS\" lfs ls-files"

