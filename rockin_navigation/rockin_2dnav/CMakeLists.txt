cmake_minimum_required(VERSION 2.8.3)
project(rockin_2dnav)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

find_package(Boost REQUIRED
  COMPONENTS
    system
)
find_package(PCL REQUIRED)


catkin_package(
#  CATKIN_DEPENDS other_catkin_pkg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ros/include
)
