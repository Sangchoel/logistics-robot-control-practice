# Logistics Robot Control Practice

ROS 2 Humble + Gazebo Classic 11 기반 로봇 제어 실습 환경  

---

## 1. 클론 & 빌드

```bash
# 워크스페이스 생성
mkdir -p ~/logistics-robot-control-practice/src
cd ~/logistics-robot-control-practice/src

# 예시: AWS Robomaker Small Warehouse World 클론
git clone https://github.com/aws-robomaker/aws-robomaker-small-warehouse-world.git

# 빌드
cd ~/logistics-robot-control-practice
colcon build --symlink-install
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
# Gazebo에서 창고 월드 실행
ros2 launch aws_robomaker_small_warehouse_world warehouse.launch.py

# TurtleBot3 시뮬레이션 실행 (예시)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
