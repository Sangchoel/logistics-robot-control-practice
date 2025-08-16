#!/usr/bin/env bash
set -euo pipefail
WS=${WS:-$HOME/ros2_course_ws}
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 기본 데모
ros2 launch tb3_multi_robot gazebo_multi_world.launch.py

# Nav2 통합 데모가 필요하면 아래 사용
# ros2 launch tb3_multi_robot gazebo_multi_nav2_world.launch.py
