cmake_minimum_required(VERSION 3.0.2)
project(limo_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs diagnostic_msgs
  image_transport cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/line_tracker.py
  nodes/boundary_from_scan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
