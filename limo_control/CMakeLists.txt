cmake_minimum_required(VERSION 3.0.2)
project(limo_control)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/cmd_mux.py
  nodes/cmd_limiter.py
  nodes/safety_stop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY params DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
