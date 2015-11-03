cmake_minimum_required(VERSION 2.8.3)
project(rockin_hardware_config)
find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rockin_calibration_data
    youbot_description
)

set(
  INSTALL_DIRS 
    rockin_youBot 
) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach()
