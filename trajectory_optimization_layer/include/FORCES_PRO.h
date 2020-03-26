#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "FORCESNLPsolver.h"
#include <cmath>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <uav_abstraction_layer/State.h>
#include <optimal_control_interface/Solver.h>


// global vars
//////////// Solver variables /////////////
const double REACHING_TOLERANCE = 2.0;
// solver output
std::vector<double> x;   
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;

/// solver inputs

geometry_msgs::PoseStamped target_pose;
std::map<int,geometry_msgs::PoseStamped> uavs_pose;
std::map<int,optimal_control_interface::Solver> uavs_trajectory;

int drone_id;
std::vector<int> drones;


// solver options
const bool target = true;
const bool multi = false;
const bool no_fly_zone = false;
const bool debug = true;
std::vector<int> priority;
geometry_msgs::TwistStamped own_velocity;
std::vector<geometry_msgs::Point> target_trajectory;
///////// solver params /////////
const int time_horizon = 100; // time horizon
const double step_size = 0.1; // seg
const int n_states_variables = 9;
const float hovering_distance = 0.5;
const int npar = 10;
const float height = 1.7;
// state vector

const int acceleration_x = 0;
const int acceleration_y = 1;
const int acceleration_z = 2;

const int position_x = 3;
const int position_y = 4;
const int position_z = 5;

const int velocity_x = 6;
const int velocity_y = 7;
const int velocity_z = 8;

// initial guess

const float u_x = 1;
const float u_y = 1;
const float u_z = 1;

const float p_x = 1;
const float p_y = 1;
const float p_z = 26;

const float v_x = 1;
const float v_y = 1;
const float v_z = 1;

// shooting actions types

const int flyover = 0;
const int orbital = 1;
const int lateral = 2;

int shooting_type = 0;

// log

std::ofstream csv_pose; // logging the trajectory
std::ofstream csv_debug; // logging the trajectory
std::ofstream csv_record; // logging the trajectory


int solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,std::vector<double> &desired_wp, std::vector<double> &desired_vel, std::vector<double> &obst, std::vector<double> &target_vel);
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
void logToCsv(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy, const std::vector<double> &vz);
