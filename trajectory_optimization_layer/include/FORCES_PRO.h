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
#define TIME_HORIZON 40


class FORCESPROsolver{
    public:
        /** \brief constructor of FORCESPROsolver();
         *  \TODO implement constructor
         */
        FORCESPROsolver();
        
        /** \brief This function fill the solver inputs and call it
        *  \param x y z vx vy vz       These are the variables where the calculated path will place
        *  \param desired_pose         Desired position
        *  \param obst                 No fly zone
        *  \param target_vel           [target_vx target_vy targe_vz] We guess velocity constant target
        *  \TODO m                     manage priorities by drones (ID)
        */
        int solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,std::array<double,TIME_HORIZON> &ax, std::array<double,TIME_HORIZON> &ay, std::array<double,TIME_HORIZON> &az, std::array<double,TIME_HORIZON> &x, std::array<double,TIME_HORIZON> &y, std::array<double,TIME_HORIZON> &z, std::array<double,TIME_HORIZON> &vx, std::array<double,TIME_HORIZON> &vy, std::array<double,TIME_HORIZON> &vz,const nav_msgs::Odometry &desired_odometry, const std::array<float,2> &obst, const std::vector<nav_msgs::Odometry> &target_trajectory, std::map<int,nav_msgs::Odometry> &uavs_pose, const int drone_id = 1, const bool target = true, const bool multi = false);
    private:
        int checkTime();

        //////////// Solver variables /////////////
        // solver options
        const int time_horizon = TIME_HORIZON;
        const bool no_fly_zone = false;
        const bool debug = true;
        std::vector<int> priority;
        ///////// solver params /////////
        const int n_states_variables = 9;
        const float hovering_distance = 0.5;
        const int npar = 10;
        const float height = 1.7;
        const float solving_rate_ = 1.0;
        const float step_size_ = 0.2;
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

        const float p_x = 4;
        const float p_y = 4;
        const float p_z = 26;

        const float v_x = 1;
        const float v_y = 1;
        const float v_z = 1;

        std::chrono::time_point<std::chrono::system_clock> start;

        //log        
        std::ofstream csv_debug; // logging the trajectory
        void saveParametersToCsv(const FORCESNLPsolver_params &params);
};






