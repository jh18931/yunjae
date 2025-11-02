cmake_minimum_required(VERSION 3.0.2)
project(ugv_sdk)

add_compile_options(-std=c++14 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs diagnostic_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ugv_sdk_node src/ugv_sdk_node.cpp)
target_link_libraries(ugv_sdk_node ${catkin_LIBRARIES})
install(TARGETS ugv_sdk_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY params launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
