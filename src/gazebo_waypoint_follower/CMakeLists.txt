cmake_minimum_required(VERSION 3.10)
project(gazebo_waypoint_follower)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

add_library(waypoint_follower SHARED src/waypoint_follower.cpp)
target_include_directories(waypoint_follower PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(waypoint_follower ${GAZEBO_LIBRARIES})
set_target_properties(waypoint_follower PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)

install(TARGETS waypoint_follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 설치 후 자동으로 GAZEBO_PLUGIN_PATH에 <install>/lib 를 prepend
ament_environment_hooks(env-hooks/gazebo_plugin_path.dsv)

ament_package()

