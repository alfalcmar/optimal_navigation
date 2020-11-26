#include<solver.h>    

NumericalSolver::Solver::Solver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> initial_guess) : solving_rate_(solving_rate),
                                                                        time_horizon_(time_horizon),
                                                                        solution_(new State[time_horizon]),                                                                
                                                                        initial_guess_(initial_guess)
{



}

int NumericalSolver::Solver::solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){

}


