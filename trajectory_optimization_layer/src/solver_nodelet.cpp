#include "solver_nodelet.h"


namespace optimal_control_interface
{

/* onInit() method //{ */

void SolverNodelet::onInit() {

    /* ros::init(_argc, _argv,"solver"); */
    ros::NodeHandle pnh = nodelet::Nodelet::getPrivateNodeHandle();
    ros::NodeHandle nh;
    const int time_horizon = 40;
    #ifdef USE_MRS_INTERFACE
    backendSolverMRS backendSolver(pnh,nh,time_horizon);
    #else
    backendSolverUAL backendSolver(pnh,nh,time_horizon);
    #endif
    backendSolver.stateMachine();
}

//}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(optimal_control_interface::SolverNodelet, nodelet::Nodelet)
