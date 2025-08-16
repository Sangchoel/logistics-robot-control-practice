#!/usr/bin/env bash
set -euo pipefail

# ===== 설정 =====
WS=${WS:-$HOME/ros2_course_ws}
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[1/8] 도구 설치"
sudo apt update
sudo apt install -y git curl python3-vcstool python3-colcon-common-extensions python3-rosdep

echo "[2/8] rosdep 초기화/업데이트"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "[3/8] ROS 2 Humble 확인"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble(우분투 22.04)이 필요합니다."
  exit 1
fi
source /opt/ros/humble/setup.bash

echo "[4/8] Gazebo 런타임(부족분은 rosdep로 추가 설치)"
sudo apt install -y gazebo libgazebo-dev

echo "[5/8] 워크스페이스 생성: $WS"
mkdir -p "$WS/src"
cd "$WS"

echo "[6/8] 강의 패키지 복사(src/)"
rsync -a --delete --exclude .git "$REPO_DIR/src/" "$WS/src/"

echo "[7/8] 시스템 의존 자동 설치(rosdep)"
rosdep install --from-paths src --rosdistro humble -y --ignore-src

echo "[8/8] 빌드(colcon)"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cat <<EOM

=== 설치 완료 ===
매 터미널에서:
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash

실행 예:
  ros2 launch tb3_multi_robot gazebo_multi_world.launch.py
  # 또는
  # ros2 launch tb3_multi_robot gazebo_multi_nav2_world.launch.py

문제 시:
  rm -rf $WS/{build,install,log} && $REPO_DIR/build.sh
EOM
