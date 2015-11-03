cmake_minimum_required(VERSION 2.8.3)
project(rockin_default_env_config)
find_package(catkin REQUIRED)

catkin_package()

install(FILES upload_param_rockin-arena.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/upload_param_rockin-arena.launch
)

install(DIRECTORY rockin-arena/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rockin-arena/
)
