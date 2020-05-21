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
#define TIME_HORIZON 40

// TODO use namespace

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
        int solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,std::array<double,TIME_HORIZON> &_ax, std::array<double,TIME_HORIZON> &_ay, std::array<double,TIME_HORIZON> &_az,std::array<double,TIME_HORIZON> &_x, std::array<double,TIME_HORIZON> &_y, std::array<double,TIME_HORIZON> &_z, std::array<double,TIME_HORIZON> &_vx, std::array<double,TIME_HORIZON> &_vy, std::array<double,TIME_HORIZON> &_vz,nav_msgs::Odometry &_desired_odometry, const std::array<float,2> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, const int _drone_id = 1, const bool _target = true, const bool _multi = false);

    private:
        void checkConstraints(nav_msgs::Odometry &desired_odometry, std::map<int,nav_msgs::Odometry> &uavs_pose);

        //////////// Solver variables /////////////
        // solver options

        const float solving_rate_ = 1.0; // solving rate (s)
        const float t_start = 0.0;
        const int drone_id_ = 1;
        const float t_end = 7.8; 
        const int N = TIME_HORIZON;
        const bool no_fly_zone = false;
        const bool debug = true;
        std::vector<int> priority;
        ///////// solver params /////////
        const double step_size = 0.2; // seg
        const int n_states_variables = 9;
        const float hovering_distance = 0.5;
        const int npar = 10;
        const float height = 1.7;
        std::ofstream csv; // logging the trajectory
        std::ofstream csv_debug; // logging the trajectory
        std::ofstream csv_record; // logging the trajectory

        LogRecord *logRecord;

        Grid *my_grid_;


};



