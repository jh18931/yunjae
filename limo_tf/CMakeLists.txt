cmake_minimum_required(VERSION 3.0.2)
project(limo_tf)

find_package(catkin REQUIRED COMPONENTS
  rospy tf2_ros geometry_msgs diagnostic_msgs std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/base_tf_broadcaster.py
  nodes/camera_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
