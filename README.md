# ROS2 Humble Course Workspace
포함: AWS Robomaker Warehouse World, TB3 멀티로봇, Gazebo Waypoint Follower(C++), Multi Robot Controller(Python)

## 요구사항
- Ubuntu 22.04 + ROS 2 Humble + gazebo classic11
- 디스크 10GB+, RAM 8GB+ 권장

## 설치(원클릭)
```bash
git clone https://github.com/Sangchoel/logistics-robot-control-practice.git
cd logistics-robot-control-practice
./bootstrap.sh

##빌드후
gedit ~/.bashrc 에서 map model, world 경로 설정


# ===== ROS 2 Humble =====
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# ===== Gazebo 11 기본 셋업(기본 경로/모델 등록) =====
if [ -f /usr/share/gazebo-11/setup.sh ]; then
  source /usr/share/gazebo-11/setup.sh
fi

# ===== 실습 워크스페이스 =====
export LRC_WS="$HOME/logistics-robot-control-practice"

# (있으면) 워크스페이스 overlay도 로드
if [ -f "$LRC_WS/install/setup.bash" ]; then
  source "$LRC_WS/install/setup.bash"
fi

# 옵션: TB3 모델
export TURTLEBOT3_MODEL=waffle

# ===== AWS Warehouse World 경로를 '기본값 뒤에 추가' =====
# 설치 prefix (ros2가 먼저 source돼 있어야 동작)
_PKGPFX="$(ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null || true)"

# RESOURCE_PATH: 패키지 루트(share) 추가
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world" ]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world"
fi

# MODEL_PATH: 모델 디렉터리 추가
if [ -d "$LRC_WS/src/aws-robomaker-small-warehouse-world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$LRC_WS/src/aws-robomaker-small-warehouse-world/models"
fi
if [ -n "$_PKGPFX" ] && [ -d "$_PKGPFX/share/aws_robomaker_small_warehouse_world/models" ]; then
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$_PKGPFX/share/aws_robomaker_small_warehouse_world/models"
fi

# OGRE 셰이더/미디어 (RTShaderLib 오류 예방)
if [ -d /usr/share/OGRE/Media ]; then
  _OGRE_ADD="/usr/share/OGRE/Media:/usr/share/OGRE/Media/RTShaderLib"
  export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:+$OGRE_RESOURCE_PATH:}${_OGRE_ADD}"
fi

# 로케일 이슈(소수점) 회피 (선택)
export LC_NUMERIC=C

## 경로 설정 검사 및 world 실행
source ~/.bashrc
echo "$GAZEBO_RESOURCE_PATH"
echo "$GAZEBO_MODEL_PATH"
echo "$OGRE_RESOURCE_PATH"

# 직접 월드 열기
gazebo --verbose "$(ros2 pkg prefix aws_robomaker_small_warehouse_world)/share/aws_robomaker_small_warehouse_world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world"

# 또는 ROS2 launch
ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse.launch.py

