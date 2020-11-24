#include<backendSolverMRS.h>
#include<backendSolverUAL.h>


int main(int _argc, char **_argv)
{
    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");
    ros::NodeHandle nh;
    const int time_horizon = 40;
    backendSolverMRS backendSolver(pnh,nh,time_horizon);
    backendSolver.stateMachine();
    return 0;
}