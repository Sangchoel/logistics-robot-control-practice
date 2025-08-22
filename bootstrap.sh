#!/usr/bin/env bash
set -euo pipefail

WS=$HOME/logistics-robot-control-practice   # 워크스페이스 = 이 레포
cd "$WS"

echo "[1/6] ROS 2 Humble 확인/소스"
[ -f /opt/ros/humble/setup.bash ] || { echo "ROS 2 Humble 필요 (/opt/ros/humble/setup.bash 없음)"; exit 1; }
# set -u 상태에서의 unbound variable 방지
set +u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source /opt/ros/humble/setup.bash
set -u

echo "[2/6] rosdep 업데이트"
rosdep update

echo "[3/6] LFS 동기화"
git -C "$WS" lfs pull || true
# (선택) 포인터 잔재 확인: grep -R "oid sha256" -n src || true

echo "[4/6] 의존성 설치(시스템 패키지)"
rosdep install --from-paths src --rosdistro humble -y --ignore-src || true
# 일부 환경에서 'ament_python' 경고가 나와도 /opt/ros/humble 내 파이썬 모듈로 빌드 가능

echo "[5/6] 빌드"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "[6/6] 완료"
echo "==============================================="
echo " 빌드 완료!"
echo "   source \"$WS/install/setup.bash\""
echo "   git -C \"$WS\" lfs ls-files | head"
echo "==============================================="

