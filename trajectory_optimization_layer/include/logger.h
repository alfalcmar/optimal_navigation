#include<fstream>
#include <nav_msgs/Odometry.h>
#include <UAVState.h>
#include <ctime>
#include <string>
#include <ros/package.h>
#include <iostream>
class backendSolver; // forward declaration

namespace SolverUtils{


class Logger{

private:
    std::ofstream file_;
    backendSolver* class_to_log_ptr_;
public:
    Logger(backendSolver* class_to_log_ptr_);

    ~Logger();

    void logging();

    void loggingCalculatedTrajectory(const int solver_success);
};

}