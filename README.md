# Logistics Robot Control Practice

ROS 2 Humble + Gazebo Classic 11 기반 다중 로봇 제어 실습 환경 

## 0) 사전 준비
Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 환경에서 아래를 **먼저 설치**하세요.

```bash
sudo apt update
# 필수 개발도구
sudo apt install -y git git-lfs curl \
  python3-vcstool python3-colcon-common-extensions python3-rosdep

# Gazebo 본체 + ROS 연동
sudo apt install -y gazebo libgazebo-dev \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# TurtleBot3 패키지(모델/가제보 시뮬)
sudo apt install -y "ros-humble-turtlebot3*"

# (선택) Nav2 / Cartographer를 쓰면 함께 설치
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-cartographer ros-humble-cartographer-ros

# git-lfs 1회 초기화
git lfs install

# rosdep 1회 초기화(이미 했다면 건너뜀)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```
## 1. 클론 & 빌드

```bash
git clone https://github.com/Sangchoel/logistics-robot-control-practice.git
cd logistics-robot-control-practice
./bootstrap.sh # rosdep 설치, colcon 빌드, 환경설정 등 자동화
```


## 2. 환경 변수 설정
아래 내용을 ~/.bashrc 마지막에 추가하세요

```bash

# ───────── ROS 2 & Gazebo base ─────────
# ROS 2 Humble
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# Gazebo Classic 11 (우분투 22.04 기본)
[ -f /usr/share/gazebo-11/setup.sh ] && source /usr/share/gazebo-11/setup.sh

# ───────── 기본 환경 ─────────
export ROS_DOMAIN_ID=30            # TURTLEBOT3 도메인
export TURTLEBOT3_MODEL=waffle     # 로봇 모델
export LC_NUMERIC=C                # Gazebo 소수점 파싱 이슈 회피
export LRC_WS="$HOME/logistics-robot-control-practice"

# ───────── Gazebo 플러그인/리소스 경로 ─────────
# Gazebo가 ROS 플러그인(.so: ros_init/factory/force_system)을 찾도록
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH:-}"

# 설치 prefix 조회(설치되어 있지 않으면 빈 문자열)
_PKGPFX="$(command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null || echo "")"

# Resource Path (world/SDF, media 등) — 소스 + 설치 모두 추가
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world"
fi

# Model Path (models 디렉터리만 — worlds 는 RESOURCE PATH로 처리)
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world/models"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world/models"
fi

# OGRE 셰이더/미디어 보강 (경고 감소)
export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:+$OGRE_RESOURCE_PATH:}/usr/share/OGRE/Media:/usr/share/OGRE/Media/RTShaderLib"


```
## 적용:

```bash

source ~/.bashrc
```

## 3. 실행 예시

```bash

# 맵만 확인
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py 

# 맵 로딩 & 멀티로봇 스폰
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py

# 멀티로봇 제어 노드 실행
ros2 run multi_robot_controller multi_robot_controller --ros-args   -p robot_namespaces:="[tb1, tb2, tb3, tb4]"   -p required_sites_yaml:="/your/path/robot_ws/src/multi_robot_controller/required_sites.yaml"   -p dry_run:=false```
