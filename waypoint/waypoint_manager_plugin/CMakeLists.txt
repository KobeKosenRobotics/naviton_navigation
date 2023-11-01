cmake_minimum_required(VERSION 3.5)
project(waypoint_manager_plugin)

###################
# compile options #
###################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -DEIGEN_RUNTIME_NO_MALLOC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fno-asm")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

#############
## Library ##
#############
# Qt settings #
find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
macro(qt_wrap_ui)
  qt5_wrap_ui(${ARGN})
endmacro()
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)
# cmake settings #
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# catkin #
set(DEPEND_ROS_PKGS
  class_loader
  pluginlib
  roscpp
  rviz
  std_msgs
  waypoint_manager_msgs
  )
find_package(catkin REQUIRED
  COMPONENTS
  ${DEPEND_ROS_PKGS}
  )

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${DEPEND_ROS_PKGS}
  DEPENDS
  )

#########
# Build #
#########
## add include directory ##
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
## add library path ##
link_directories(${catkin_LIBRARY_DIRS})

# source codes #
set(HEADERS
  include/waypoint_manager_plugin/recorder_panel.h
  )

qt5_wrap_ui(UIC_FILES
  src/ui/recorder_panel.ui
  )

set(SOURCE_FILES
  src/recorder_panel.cpp
  )

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADERS} ${UIC_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_include_directories(${PROJECT_NAME} PRIVATE "${OGRE_PREFIX_DIR}/include")

###########
# Install #
###########
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
