cmake_minimum_required(VERSION 2.8.3)
project(rockin_arm_cartesian_control)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs rockin_manipulation_utils brics_actuator tf urdf)
#find_package(eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    rockin_manipulation_utils
)

include_directories(
 ${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include
 ${EIGEN_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
)

add_executable(rockin_arm_cartesian_control
    ros/src/rockin_arm_cartesian_control_node.cpp
    common/src/rockin_arm_cartesian_control.cpp)
    
target_link_libraries(rockin_arm_cartesian_control
  ${catkin_LIBRARIES} 
  ${orocos_kdl_LIBRARIES}  
)


add_dependencies(rockin_arm_cartesian_control 
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS rockin_arm_cartesian_control rockin_arm_cartesian_control
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
