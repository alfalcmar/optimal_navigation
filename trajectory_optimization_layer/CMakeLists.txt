cmake_minimum_required(VERSION 2.8.3)
project(optimal_control_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake/Modules/")

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
find_package(ACADO REQUIRED)
if(NOT ACADO_FOUND)
  message(FATAL_ERROR " ACADO *not* found")
endif()



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
  DEPENDS ACADO
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
  ${ACADO_INCLUDE_DIRS}

)





## Declare a C++ library
add_library(backend_solver_library
  src/backendSolver.cpp
)
## Declare a C++ library
add_library(ACADO_library
  src/ACADO.cpp
)

ADD_DEFINITIONS( -std=c++11 )

add_executable(optimal_control_interface_node src/solver_node.cpp)



target_link_libraries(optimal_control_interface_node backend_solver_library ACADO_library)


target_link_libraries(optimal_control_interface_node 
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES} ${ACADO_SHARED_LIBRARIES}) 



add_dependencies(optimal_control_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


