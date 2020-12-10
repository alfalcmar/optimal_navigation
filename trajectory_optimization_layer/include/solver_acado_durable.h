#ifndef SOLVER_ACADO_DURABLE_H
#define SOLVER_ACADO_DURABLE_H

#include <solver_acado.h>
#include <ros/ros.h>

namespace NumericalSolver{

class SolverDurable : public NumericalSolver::ACADOSolver{
    public:
        SolverDurable(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &initial_guess);
        int solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);
    private: 
        bool getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving, const int _closest_point);
        int minDistance(const UavState &_uavs_pose);
};
}
#endif