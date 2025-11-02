cmake_minimum_required(VERSION 3.0.2)
project(limo_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  diagnostic_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/check_devices.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
