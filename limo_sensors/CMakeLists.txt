cmake_minimum_required(VERSION 3.0.2)
project(limo_sensors)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs std_msgs diagnostic_msgs image_transport cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/camera_node.py
  nodes/lidar_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
