cmake_minimum_required(VERSION 2.8.3)
project(rockin_navigation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
    roscpp
    rospy
    std_msgs
)

add_action_files(
  DIRECTORY action
  FILES OrientToBase.action
)

add_service_files(
  FILES
    MoveRelative.srv
    GetBasePose.srv
)

generate_messages(
  DEPENDENCIES 
    actionlib_msgs 
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime    
    std_msgs
)
