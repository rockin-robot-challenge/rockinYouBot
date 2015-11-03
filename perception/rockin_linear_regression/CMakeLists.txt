cmake_minimum_required(VERSION 2.8.3)
project(rockin_linear_regression)

find_package(catkin REQUIRED
  COMPONENTS
    rockin_perception_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    rockin_perception_msgs 
)

include_directories(
  common/include/
  ros/include/
  ${catkin_INCLUDE_DIRS}
)

add_executable(rockin_laser_scan_linear_regression_node 
  ros/src/rockin_laser_scan_linear_regression_node.cpp
  ros/src/rockin_laser_scan_linear_regression_util.cpp
  common/src/rockin_laser_scan_linear_regression.cpp
)
add_dependencies(rockin_laser_scan_linear_regression_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rockin_laser_scan_linear_regression_node
  ${catkin_LIBRARIES}
)

install(TARGETS rockin_laser_scan_linear_regression_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch/
)
