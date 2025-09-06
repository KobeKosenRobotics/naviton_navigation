cmake_minimum_required(VERSION 3.0.2)
project(safety_planner)

cmake_minimum_required(VERSION 3.0.2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(safety_planner_node src/safety_planner_node.cpp)
target_link_libraries(safety_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS safety_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
