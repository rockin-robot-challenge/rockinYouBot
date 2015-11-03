cmake_minimum_required(VERSION 2.8.3)
project(rockin_bringup_sim)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    rockin_gazebo_worlds   
    rockin_arm_cartesian_control
    rockin_default_robot_config
    rockin_hardware_config
    robot_state_publisher
    youbot_gazebo_control
)

install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
)
install(FILES robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
