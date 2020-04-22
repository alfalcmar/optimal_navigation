#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
// #include <cmath>
// #include <uav_abstraction_layer/GoToWaypoint.h>
// #include <boost/thread/thread.hpp>
// #include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <Eigen/Eigen>
// #include <time.h>
// #include <geometry_msgs/Point.h>
// #include <geometry_msgs/Point.h>
// #include <thread>         // std::thread, std::this_thread::sleep_for
// #include <uav_abstraction_layer/State.h>
// #include <optimal_control_interface/Solver.h>
#include <acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
USING_NAMESPACE_ACADO


class ACADOsolver{
    public:
        ACADOsolver();
        /** \brief This function fill the solver inputs and call it
        *  \param x y z vx vy vz       These are the variables where the calculated path will place
        *  \param desired_pose         Desired position
        *  \param obst                 No fly zone
        *  \param target_vel           [target_vx target_vy targe_vz] We guess velocity constant target
        *  \TODO m                     manage priorities by drones (ID)
        */
        int solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,nav_msgs::Odometry &desired_odometry, const std::array<float,2> &obst, const std::vector<nav_msgs::Odometry> &target_trajectory, std::map<int,nav_msgs::Odometry> &uavs_pose, const int drone_id = 1, const bool target = true, const bool multi = false);

    private:
        void checkConstraints(nav_msgs::Odometry &desired_odometry, std::map<int,nav_msgs::Odometry> &uavs_pose);

        //////////// Solver variables /////////////
        // solver options
        const float t_start = 0.0;
        const int drone_id_ = 1;
        const float t_end = 8.0; 
        const int N = 39;
        const bool no_fly_zone = false;
        const bool debug = true;
        std::vector<int> priority;
        ///////// solver params /////////
        const double step_size = 0.2; // seg
        const int n_states_variables = 9;
        const float hovering_distance = 0.5;
        const int npar = 10;
        const float height = 1.7;
        std::ofstream csv_pose; // logging the trajectory
        std::ofstream csv_debug; // logging the trajectory
        std::ofstream csv_record; // logging the trajectory

        LogRecord *logRecord;


};



