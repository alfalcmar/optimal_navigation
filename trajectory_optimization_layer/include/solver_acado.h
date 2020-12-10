#ifndef SOLVERACADO_H
#define SOLVERACADO_H

#include<solver.h>

namespace NumericalSolver{

class ACADOSolver : public Solver{

private:
    bool logACADOvars();
    virtual bool getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving);

public:
    ACADOSolver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &intial_guess);

    /** \brief This function fill the solver inputs and call it
    *  \param x y z vx vy vz       These are the variables where the calculated path will place
    *  \param desired_pose         Desired position
    *  \param obst                 No fly zone
    *  \param target_vel           [target_vx target_vy targe_vz] We guess velocity constant target
    *  \TODO m                     manage priorities by drones (ID)
    */
    virtual int solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);

};

}

#endif