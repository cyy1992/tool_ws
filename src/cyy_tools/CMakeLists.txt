cmake_minimum_required(VERSION 2.8.3)
project(cyy_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/modules)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Glog REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  rosbag
  roscpp
  roslib
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
  poslvx
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cyy_tools
#  CATKIN_DEPENDS cv_bridge eigen_conversions rosbag roscpp roslib tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

add_executable(merge_bags_node src/merge_bag.cpp)
target_link_libraries(merge_bags_node
  ${catkin_LIBRARIES}
)

install(TARGETS merge_bags_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION } / ${PROJECT_NAME }
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)