cmake_minimum_required(VERSION 2.8.3)
project(rockin_default_robot_config)
find_package(catkin REQUIRED)

catkin_package()

install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
