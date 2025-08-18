# Logistics Robot Control Practice

ROS 2 Humble + Gazebo Classic 11 기반 다중 로봇 제어 실습 환경  

---

## 1. 클론 & 빌드

```bash
git clone https://github.com/Sangchoel/logistics-robot-control-practice.git
cd logistics-robot-control-practice
./bootstrap.sh
```


## 2. 환경 변수 설정
아래 내용을 ~/.bashrc 마지막에 추가하세요

```bash

아래 내용을 ~/.bashrc 마지막에 추가하세요.
# 로봇 모델 (TurtleBot3 Waffle)
export TURTLEBOT3_MODEL=waffle

# 워크스페이스 경로
export LRC_WS="$HOME/logistics-robot-control-practice"

# --------- Gazebo 경로 추가 ----------

# 설치 prefix 확인
_PKGPFX="$(ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null || true)"

# Resource Path
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world"
fi

# Model Path
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world/models"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world/models"
fi

# OGRE 셰이더/미디어
export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:+$OGRE_RESOURCE_PATH:}/usr/share/OGRE/Media:/usr/share/OGRE/Media/RTShaderLib"

# 로케일 설정
export LC_NUMERIC=C
```
## 적용:

```bash

source ~/.bashrc
```

## 3. 실행 예시

```bash

# map 확인
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py 

# map 로딩 & 멀티로봇 스폰
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py

# 멀티로봇 제어 노드 실행
ros2 run multi_robot_controller multi_robot_controller --ros-args   -p robot_namespaces:="[tb1, tb2, tb3, tb4]"   -p required_sites_yaml:="/home/osc/robot_ws/src/multi_robot_controller/required_sites.yaml"   -p dry_run:=false```
