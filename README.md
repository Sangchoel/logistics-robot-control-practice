# Logistics Robot Control Practice

이 저장소는 물류 로봇 제어 실습을 위한 워크스페이스입니다.  
아래 과정을 따라 환경을 준비하세요.

---

## 1. 워크스페이스 클론 및 부트스트랩

```bash
git clone https://github.com/Sangchoel/logistics-robot-control-practice.git
cd logistics-robot-control-practice
./bootstrap.sh

2. 환경 변수 설정 (.bashrc)

~/.bashrc 맨 아래에 다음 내용을 추가합니다:

# ----------------- Logistics Robot Control Practice -----------------
export TURTLEBOT3_MODEL=waffle
export LRC_WS="$HOME/logistics-robot-control-practice"

# --------- Gazebo 경로 추가 ----------
_PKGPFX="$(ros2 pkg prefix aws_robomaker_small_warehouse_world 2>/dev/null || true)"

# RESOURCE_PATH: 패키지 루트 추가
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

# OGRE 셰이더/미디어 경로 (Shader 에러 방지)
export OGRE_RESOURCE_PATH="${OGRE_RESOURCE_PATH:+$OGRE_RESOURCE_PATH:}/usr/share/OGRE/Media:/usr/share/OGRE/Media/RTShaderLib"

# 로케일 이슈 방지 (소수점 문제)
export LC_NUMERIC=C

설정 후 적용:

source ~/.bashrc

3. 빌드

cd $LRC_WS
colcon build --symlink-install

4. 실행

예시: Gazebo 시뮬레이터 실행

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

(실습 환경에 맞게 런치파일 선택)
