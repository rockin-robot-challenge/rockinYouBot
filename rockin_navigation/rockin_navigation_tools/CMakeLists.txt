cmake_minimum_required(VERSION 2.8.3)
project(rockin_navigation_tools)
find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  ros/scripts/rockin_navigation_goals_as_marker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
