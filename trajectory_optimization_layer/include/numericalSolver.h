#include <vector>
#include <acado_optimal_control.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <chrono>
#include <acado/utils/acado_utils.hpp>
#include <nav_msgs/Odometry.h>
#include <memory>

USING_NAMESPACE_ACADO

#define TIME_HORIZON 40

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
    int time_horizon_;
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


public:
    std::unique_ptr<double[]> x_ptr_;  
    std::unique_ptr<double[]> y_ptr_;
    std::unique_ptr<double[]> z_ptr_;
    std::unique_ptr<double[]> vx_ptr_;
    std::unique_ptr<double[]> vy_ptr_;
    std::unique_ptr<double[]> vz_ptr_;
    std::unique_ptr<double[]> ax_ptr_;
    std::unique_ptr<double[]> ay_ptr_;
    std::unique_ptr<double[]> az_ptr_;


    Solver(const float solving_rate, const int time_horizon);
    virtual int solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess, nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);

};


class ACADOSolver : public Solver{

private:
    bool logACADOvars();
    bool getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving);

public:
    ACADOSolver(const float solving_rate, const int time_horizon);

    /** \brief This function fill the solver inputs and call it
    *  \param x y z vx vy vz       These are the variables where the calculated path will place
    *  \param desired_pose         Desired position
    *  \param obst                 No fly zone
    *  \param target_vel           [target_vx target_vy targe_vz] We guess velocity constant target
    *  \TODO m                     manage priorities by drones (ID)
    */
    int solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess, nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);

};

// class FORCESProSolver : public Solver{


// };

}