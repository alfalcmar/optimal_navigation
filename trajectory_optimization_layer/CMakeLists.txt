cmake_minimum_required(VERSION 2.8.3)
project(optimal_control_interface)

## Compile as C++11, supported in ROS Kinetic and newer
ADD_DEFINITIONS( -std=c++17 )
add_compile_options(-std=c++17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w")

# set(CMAKE_BUILD_TYPE Release)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} "cmake/Modules/" )

set(ENV{ACADO} "true")


set(MRS_INTERFACE ON)


find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  tf
  roslaunch
  std_msgs
  std_srvs
  #uav_abstraction_layer
  nav_msgs
  shot_executer
  safe_corridor_generator 
  decomp_ros_utils
	pcl_ros
	pcl_conversions
  geometry_msgs
)


if( MRS_INTERFACE)
  ADD_DEFINITIONS(-DUSE_MRS_INTERFACE)
  find_package(formation_church_planning REQUIRED
              mrs_msgs REQUIRED)
endif()



find_package(PythonLibs 2.7)

#Find Eigen libraries
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})

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
 CATKIN_DEPENDS roscpp rospy tf std_msgs std_srvs
 LIBRARIES solver_library SolverNodelet 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${safe_corridor_generator_INCLUDE_DIRS}
  solver
  solver/FORCESNLPsolver/lib
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(DEFINED ENV{ACADO})
    include_directories(  ${ACADO_INCLUDE_DIRS})


  add_library(solver_library
    src/solver.cpp
    src/solver_acado.cpp
    src/UAVState.cpp
    src/logger.cpp
    src/backendSolver.cpp
  )

target_link_libraries(solver_library
${catkin_LIBRARIES}
)

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

add_library(SolverNodelet 
  src/solver_nodelet.cpp
  src/backendSolverMRS.cpp
  )

target_link_libraries(SolverNodelet
  solver_library
  ${catkin_LIBRARIES} 
  ${EXTRALIB_BIN} 
  ${PYTHON_LIBRARIES} 
  ${Eigen3_LIBRARIES} 
  ${ACADO_SHARED_LIBRARIES}
  ${JPS3D_LIBRARIES}
  ${PCL_LIBRARIES}
  ${DECOMP_UTIL_LIBRARIES}
)

add_dependencies(SolverNodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

endif()


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
