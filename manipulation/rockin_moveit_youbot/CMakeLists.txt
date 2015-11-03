cmake_minimum_required(VERSION 2.8.3)
project(rockin_moveit_youbot)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rockin_moveit_rockin_youBot
)

install(DIRECTORY rockin_youBot
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rockin_youBot
)
