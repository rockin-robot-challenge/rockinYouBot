cmake_minimum_required(VERSION 2.8.3)
project(rockin_relative_base_controller)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf  
)

generate_dynamic_reconfigure_options(
  ros/config/RelativeBaseController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rockin_relative_base_controller_node
  ros/src/rockin_relative_base_controller_node.cpp 
)
add_dependencies(rockin_relative_base_controller_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rockin_relative_base_controller_node
  ${catkin_LIBRARIES}
)


add_executable(rockin_relative_base_controller_mockup
  ros/mockup/rockin_relative_base_controller_mockup.cpp
)
add_dependencies(rockin_relative_base_controller_mockup
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rockin_relative_base_controller_mockup
  ${catkin_LIBRARIES}
)


install(TARGETS rockin_relative_base_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rockin_relative_base_controller_mockup
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
