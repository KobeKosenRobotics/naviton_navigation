cmake_minimum_required(VERSION 3.0.2)
project(joy2twist)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/joy2twist_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
