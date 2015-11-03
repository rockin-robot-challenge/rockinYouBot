cmake_minimum_required(VERSION 2.8.3)
project(rockin_base_placement)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    geometry_msgs
    rockin_perception_msgs
    rockin_navigation_msgs
    roscpp
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rockin_perception_msgs
    rockin_navigation_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rockin_base_placement_node 
  ros/src/rockin_base_placement_node.cpp
)
add_dependencies(rockin_base_placement_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rockin_base_placement_node
  ${catkin_LIBRARIES}
)

add_executable(rockin_basescan_orientation_test 
  ros/tools/rockin_basescan_orientation_test.cpp
)
add_dependencies(rockin_basescan_orientation_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rockin_basescan_orientation_test
  ${catkin_LIBRARIES}
)

install(TARGETS rockin_base_placement_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch/
)
