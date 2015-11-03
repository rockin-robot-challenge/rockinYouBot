cmake_minimum_required(VERSION 2.8.3)
project(rockin_bringup)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    hokuyo_node
    rockin_default_env_config
    rockin_arm_cartesian_control
    rockin_default_robot_config
    rockin_gripper_services
    rockin_hardware_config
    rockin_teleop
    robot_state_publisher
    tf
    youbot_driver_ros_interface
)

set(
  INSTALL_DIRS 
    components
    robots
) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

install(FILES robot.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  ) 
