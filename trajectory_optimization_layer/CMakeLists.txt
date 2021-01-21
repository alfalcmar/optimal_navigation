cmake_minimum_required(VERSION 3.0.2)
project(optimal_control_interface)

## Compile as C++11, supported in ROS Kinetic and newer
ADD_DEFINITIONS( -std=c++17 )
add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "-w")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake/Modules/")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

set(ENV{ACADO} "true")


set(MRS_INTERFACE ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  #uav_abstraction_layer
  nav_msgs
  shot_executer
  roslib
  safe_corridor_generator
  decomp_ros_utils 
)

#MRS packages
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})


if( MRS_INTERFACE)
  ADD_DEFINITIONS(-DUSE_MRS_INTERFACE)
  find_package(catkin REQUIRED COMPONENTS
              formation_church_planning REQUIRED
              mrs_msgs REQUIRED
              shot_executer)
endif()



find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)


if(DEFINED ENV{ACADO})
  find_package(ACADO REQUIRED)
  if(NOT ACADO_FOUND)
    message(FATAL_ERROR " ACADO *not* found")
  endif()
endif()


link_directories(${ACADO_LIBRARY_DIRS})

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
  LIBRARIES solver_library
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs roslib 

  #DEPENDS ACADO
)


#roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${safe_corridor_generator_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs #uav_abstraction_layer
  lib
  solver
  solver/FORCESNLPsolver/lib
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if(DEFINED ENV{ACADO})
    include_directories(  ${ACADO_INCLUDE_DIRS})


  add_library(solver_library
    src/solver.cpp
    src/solver_acado.cpp
    src/UAVState.cpp
    src/logger.cpp
    src/backendSolver.cpp
  )
endif()

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(solver_library
${catkin_LIBRARIES}
)

if(DEFINED ENV{FORCES})
  set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/solver/FORCESNLPsolver/lib/libFORCESNLPsolver.so)

  add_library(FORCES_PRO_library src/FORCES_PRO.cpp 
   solver/FORCESNLPsolver_casadi2forces.c 
   solver/FORCESNLPsolver_model_1.c
   solver/FORCESNLPsolver_model_40.c
  )
endif()

if( MRS_INTERFACE )
  add_executable(optimal_control_interface_node src/solver_node.cpp src/backendSolverMRS.cpp)
else()
  add_executable(optimal_control_interface_node src/solver_node.cpp src/backendSolverUAL.cpp)
endif()

if(DEFINED ENV{ACADO})
  target_link_libraries(optimal_control_interface_node solver_library) # FORCES_PRO_library)
endif()



if(DEFINED ENV{FORCES})
    add_definitions(-FORCES)
    target_link_libraries(optimal_control_interface_node FORCES_PRO_library)
endif()

target_link_libraries(optimal_control_interface_node 
${catkin_LIBRARIES} ${EXTRALIB_BIN} ${PYTHON_LIBRARIES} ${Eigen3_LIBRARIES} ${ACADO_SHARED_LIBRARIES}) 


add_dependencies(optimal_control_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


unset(MRS_INTERFACE) 
