#include<solver.h>    

NumericalSolver::Solver::Solver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> initial_guess, 
                                const std::shared_ptr<safe_corridor_generator::SafeCorridorGenerator> _safe_corridor_generator_ptr) : solving_rate_(solving_rate),
                                                                                                                                      time_horizon_(time_horizon),
                                                                                                                                      solution_(new State[time_horizon]),                                                                
                                                                                                                                      initial_guess_(initial_guess),
                                                                                                                                      safe_corridor_generator_(_safe_corridor_generator_ptr)
{



}

int NumericalSolver::Solver::solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){

}

vec_Vec3f NumericalSolver::Solver::pathFromStartToEnd(const Vec3f &start, const Vec3f &end){

    vec_Vec3f path;

    Vec3f vel = MAX_VEL_XY*(end-start)/(end-start).norm(); // TODO: separate xy vel and z vel

    path.push_back(start);
    for (int i = 1; i<time_horizon_;i++){
        path.push_back(path[i-1]+vel*step_size);
    }
}
