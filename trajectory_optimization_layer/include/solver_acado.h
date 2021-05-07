#ifndef SOLVERACADO_H
#define SOLVERACADO_H

#include<solver.h>

namespace NumericalSolver{

class ACADOSolver : public Solver{

private:
    bool logACADOvars();
    bool getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving);
    void polyhedronsToACADO(OCP &_ocp, const vec_E<Polyhedron<3>> &_vector_of_polyhedrons, const vec_Vec3f &_initial_path, DifferentialState &_px, DifferentialState &_py, DifferentialState &_pz);
    int mpc(ros::Publisher &pub_path_, ros::Publisher &pub_corridor_polyhedrons_, const std::map<int,UavState> &_uavs_pose, const int _drone_id);


public:
    ACADOSolver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &intial_guess, const std::shared_ptr<safe_corridor_generator::SafeCorridorGenerator> _safe_corridor_generator_ptr);

    /** \brief This function fill the solver inputs and call it
    *  \param x y z vx vy vz       These are the variables where the calculated path will place
    *  \param desired_pose         Desired position
    *  \param obst                 No fly zone
    *  \param target_vel           [target_vx target_vy targe_vz] We guess velocity constant target
    *  \TODO m                     manage priorities by drones (ID)
    */
    int solverFunction(nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, ros::Publisher &pub_path_, ros::Publisher &pub_corridor_polyhedrons_, float time_initial_position = 0, bool first_time_solving = true, const int _drone_id = 1, const bool _target = true, const bool _multi = false);
    bool testPolyhedronConstraints(const std::vector<geometry_msgs::PoseStamped> &_path, const vec_E<Polyhedron<3>> &_polyhedrons);
    nav_msgs::Path calculatePath(const Vec2f &start_pose, const Vec2f &final_pose, const State &_uav_state, const std::vector<nav_msgs::Odometry> &_target_trajectory);
    std::vector<double> orientation(const std::vector<nav_msgs::Odometry> &_target_trajectory, float actual_heading, const std::unique_ptr<State[]> &_solution);

};

}

#endif