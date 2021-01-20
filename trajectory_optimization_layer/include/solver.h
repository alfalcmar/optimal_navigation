#ifndef NUMERICALSOLVER_H_
#define NUMERICALSOLVER_H_

#include <vector>
#include <acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <chrono>
#include <acado/utils/acado_utils.hpp>
#include <nav_msgs/Odometry.h>
#include <memory>
#include <UAVState.h>
#include <safe_corridor_generator/safe_corridor_generator.h>
USING_NAMESPACE_ACADO

namespace NumericalSolver{

class Solver{

private:

protected:
    const float t_start = 0.0;
    const float t_end = 7.8; 
    const float CAMERA_PITCH = 0.1;
    const float Z_RELATIVE_TARGET_DRONE = 1.5;  /*! height of the drone with respect to the target */
    float solving_rate_; // solving rate (s)
    const bool no_fly_zone = false;
    const bool debug = true;
    std::vector<int> priority;      /*! drone priority to avoid others*/
    const double step_size = 0.2; // seg
    const int n_states_variables = 9;
    const int offset_= 5; /**! start solving from the fith point of the trajectory */
    int solver_success_ = false;
    const float MAX_ACC = 1.0;
    const float MAX_VEL_XY = 1;
    const float MAX_VEL_Z = 0.5;
    const float W_PX_N = 0.1;
    const float W_PY_N = 0.1;
    const float W_AX = 1;
    const float W_AY = 1;
    const float W_AZ = 1;
    const float W_SLACK = 5;
    
    std::shared_ptr<State[]> initial_guess_;
    const int time_horizon_;


public:

    std::unique_ptr<State[]> solution_;

    Solver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> initial_guess);
    virtual int solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);
};

}

#endif