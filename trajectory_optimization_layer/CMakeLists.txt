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
  mrs_msgs
  formation_church_planning
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
  LIBRARIES backend_solver_library
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs uav_abstraction_layer
)

#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  LIBRARIES backend_solver_library
  lib
  solver
  solver/FORCESNLPsolver/lib
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)




#set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libFORCESNLPsolver.so)
set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/solver/FORCESNLPsolver/lib/libFORCESNLPsolver.so)



add_library(optimal_control_interface_library src/FORCES_PRO.cpp 
solver/FORCESNLPsolver_casadi2forces.c 
solver/FORCESNLPsolver_model_1.c
solver/FORCESNLPsolver_model_40.c
)

## Declare a C++ library
add_library(backend_solver_library
  src/backendSolver.cpp
)


add_executable(optimal_control_interface_node src/solver_node.cpp)




target_link_libraries(optimal_control_interface_node backend_solver_library optimal_control_interface_library)


target_link_libraries(optimal_control_interface_node 
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES})



add_dependencies(optimal_control_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


