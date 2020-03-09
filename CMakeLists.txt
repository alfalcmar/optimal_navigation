cmake_minimum_required(VERSION 2.8.3)
project(optimal_control_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  uav_abstraction_layer
  nav_msgs
  multidrone_msgs
)
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)



add_message_files(
  FILES
  Solver.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs uav_abstraction_layer multidrone_msgs
)

#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  lib
  solver
  solver/FORCESNLPsolver/lib
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)




#set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libFORCESNLPsolver.so)
set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/solver/FORCESNLPsolver/lib/libFORCESNLPsolver.so)



add_library(optimal_control_interface_library src/optimal_control_interface.cpp 
solver/FORCESNLPsolver_casadi2forces.c 
solver/FORCESNLPsolver_model_1.c
solver/FORCESNLPsolver_model_100.c
)

add_executable(optimal_control_interface_node src/main.cpp)
add_executable(trajectory_follower_node src/trajectory_follower.cpp)
add_executable(shot_executer_node src/shot_executer.cpp)


target_link_libraries(optimal_control_interface_node optimal_control_interface_library)


target_link_libraries(optimal_control_interface_node 
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})
target_link_libraries(trajectory_follower_node
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})
target_link_libraries(shot_executer_node
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})

add_dependencies(optimal_control_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_follower_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(shot_executer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
