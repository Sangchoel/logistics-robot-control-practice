#!/usr/bin/env bash
set -euo pipefail
WS=${WS:-$HOME/ros2_course_ws}
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
