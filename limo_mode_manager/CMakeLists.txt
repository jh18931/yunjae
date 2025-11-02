cmake_minimum_required(VERSION 3.0.2)
project(limo_mode_manager)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/mode_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
