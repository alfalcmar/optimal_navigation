#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "FORCESNLPsolver.h"
#include "matplotlibcpp.h"
#include <cmath>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <multidrone_msgs/DirectorEvent.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <geometry_msgs/Point.h>
#include <multidrone_msgs/SolvedTrajectory.h>
#include <geometry_msgs/Point.h>
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <multidrone_msgs/ShootingAction.h>
#include <multidrone_msgs/ShootingType.h>
#include <uav_abstraction_layer/State.h>


// global vars
//////////// Solver variables /////////////

// solver output
std::vector<double> x;   
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;

/// solver inputs
std::vector<double> desired_wp{4,4,1}; 
std::vector<double> desired_vel{0,0,0};
std::vector<double> obst{0,0};
std::vector<double> target_vel = {0, 0};

float solver_rate;

bool target = false;
bool multi = false;
bool no_fly_zone = true;


bool shot_executer_action = false;
bool debug = true;
ros::Subscriber uav_state_sub;
ros::Subscriber target_array_sub;
std::map<int, ros::Subscriber> drone_pose_sub;
std::map<int, ros::Subscriber> drone_trajectory_sub;
std::map<int, bool> has_poses; //map[0] -> target
std::map<int,geometry_msgs::PoseStamped> uavs_pose;
std::map<int,std::vector<geometry_msgs::Point>> uavs_trajectory;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped target_init;
geometry_msgs::TwistStamped own_velocity;
ros::ServiceClient go_to_waypoint_srv_;
ros::Subscriber sub_position;
ros::Subscriber sub_velocity;
ros::Subscriber desired_pose_sub;
ros::Publisher path_rviz_pub;
ros::Publisher target_path_rviz_pub;
ros::Publisher path_no_fly_zone;
ros::Publisher desired_pose_publisher;
ros::Publisher solved_trajectory_pub;
clock_t initial_time;
std::vector<int> drones;
std::vector<int> priority;
int drone_id;
std::vector<double> target_final_pose{40.0,-42.0}; 
std::map<int,bool> trajectory_solved_received;
std::vector<double> f_pose{0.0,0.0,0.0};
uav_abstraction_layer::State ual_state;

const double target_vel_module = 0.5;
float shooting_duration = 60;
std::map<std::string, float> shooting_parameters;
uint8_t shooting_action_type;
std::thread shooting_action_thread;
bool shooting_action_running = false;
bool stop_current_shooting = false;
std::vector<geometry_msgs::Point> target_trajectory;

geometry_msgs::PoseStamped desired_pose;
///////// solver params /////////
const int time_horizon = 100; // time horizon
const double step_size = 0.1; // seg
const int n_states_variables = 9;
const float hovering_distance = 0.5;
const int npar = 14;

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
void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z, std::vector<double> &desired_wps);
void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]);
void publishDesiredPoint(double x, double y,double z);
void publishTrajectory(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy,const std::vector<double> &vz);
