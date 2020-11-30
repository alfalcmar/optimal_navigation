cmake_minimum_required(VERSION 2.8.3)
project(shot_executer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
#  uav_abstraction_layer
  nav_msgs
  geometry_msgs
  mavros_msgs
)
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  DesiredShot.msg
)

add_service_files(
  FILES
  ShootingAction.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES shot_executer_library
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs ##uav_abstraction_layer 
)

#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  LIBRARIES shot_executer_library

  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

## Declare a C++ library
add_library(shot_executer_library
  src/shot_executer.cpp
)




add_executable(shot_executer_node src/shot_executer_node.cpp src/shot_executer_MRS.cpp)

target_link_libraries(shot_executer_node shot_executer_library
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})

add_dependencies(shot_executer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
