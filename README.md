# Logistics Robot Control Practice

ROS 2 Humble + Gazebo Classic 11 기반 다중 로봇 제어 실습 환경  

## 0) 사전 준비
- Ubuntu 22.04 + ROS 2 Humble 설치
- Gazebo Classic 11
- NVIDIA 그래픽 드라이버 

## 1. 클론 & 빌드

```bash
git clone https://github.com/Sangchoel/logistics-robot-control-practice.git
cd logistics-robot-control-practice
./bootstrap.sh # rosdep 설치, colcon 빌드, 환경설정 등 자동화
```


## 2. 환경 변수 설정
아래 내용을 ~/.bashrc 마지막에 추가하세요

```bash

아래 내용을 ~/.bashrc 마지막에 추가하세요.
# ---------- ROS 2 & Gazebo ----------
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo-11/setup.sh ] && source /usr/share/gazebo-11/setup.sh

# 로봇 모델 (TurtleBot3 Waffle)
export TURTLEBOT3_MODEL=waffle

# 워크스페이스 경로
export LRC_WS="$HOME/logistics-robot-control-practice"

# 설치 prefix 확인(설치/소스 어느 쪽이든 대응)
_PKGPFX="$(ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null || true)"

# --------- Gazebo 경로 추가 ----------
# Resource Path (world/SDF, media 등)
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world"
fi

# Model Path (※ models 디렉터리만 추가 — worlds 넣지 않기)
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world/models"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world/models"
fi

# OGRE 셰이더/미디어 보강 (셰이더 경고 방지)
export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:+$OGRE_RESOURCE_PATH:}/usr/share/OGRE/Media:/usr/share/OGRE/Media/RTShaderLib"

# Gazebo에서 소수점 파싱 문제 방지
export LC_NUMERIC=C

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
ros2 run multi_robot_controller multi_robot_controller --ros-args   -p robot_namespaces:="[tb1, tb2, tb3, tb4]"   -p required_sites_yaml:="/home/osc/robot_ws/src/multi_robot_controller/required_sites.yaml"   -p dry_run:=false```
