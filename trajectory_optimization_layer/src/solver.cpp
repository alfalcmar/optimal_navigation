#include<solver.h>    

NumericalSolver::Solver::Solver(const float solving_rate, const int time_horizon) : solving_rate_(solving_rate),
                                                                        time_horizon_(time_horizon),
                                                                        x_ptr_(new double[time_horizon]{0.0}),
                                                                        y_ptr_(new double[time_horizon]{0.0}),
                                                                        z_ptr_(new double[time_horizon]{0.0}),
                                                                        vx_ptr_(new double[time_horizon]{0.0}),
                                                                        vy_ptr_(new double[time_horizon]{0.0}),
                                                                        vz_ptr_(new double[time_horizon]{0.0}),
                                                                        ax_ptr_(new double[time_horizon]{0.0}),
                                                                        ay_ptr_(new double[time_horizon]{0.0}),
                                                                        az_ptr_(new double[time_horizon]{0.0})
{



}

int NumericalSolver::Solver::solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){

}


