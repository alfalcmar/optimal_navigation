#include<backendSolver.h>

int main(int _argc, char **_argv)
{
    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");
    ros::NodeHandle nh;
    backendSolverMRS backendSolver(pnh,nh);
    backendSolver.stateMachine();
    return 0;
}