#ifdef USE_MRS_INTERFACE
#include<backendSolverMRS.h>
#else
#include<backendSolverUAL.h>
#endif


int main(int _argc, char **_argv)
{
    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");
    ros::NodeHandle nh;
    const int time_horizon = 40;
    #ifdef USE_MRS_INTERFACE
    backendSolverMRS backendSolver(pnh,nh,time_horizon);
    #else
    backendSolverUAL backendSolver(pnh,nh,time_horizon);
    #endif
    backendSolver.stateMachine();
    return 0;
}