
#include <solver_acado_durable.h>

using namespace NumericalSolver;

SolverDurable::SolverDurable(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &initial_guess) : ACADOSolver::ACADOSolver(solving_rate, time_horizon, initial_guess){
  std::cout<<"constructor solver durable "<<std::endl;   
}

int SolverDurable::minDistance(const UavState &_uavs_pose){
  int   nearest_point    = 0;
  float nearest_distance = INFINITY;
  float point_distance   = 0;
  for (int i = 0; i < time_horizon_; i++) {
    point_distance = sqrt(pow((solution_[i].pose.x - _uavs_pose.state.pose.x), 2) + pow((solution_[i].pose.y - _uavs_pose.state.pose.y), 2) +
                          pow((solution_[i].pose.z - _uavs_pose.state.pose.z), 2));
    if (point_distance < nearest_distance) {
      nearest_distance = point_distance;
      nearest_point    = i;
    }
  }
  return nearest_point;
}

int SolverDurable::solverFunction( nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){
    DifferentialState px_,py_,pz_,vx_,vy_,vz_;
    //DifferentialState   dummy;  // dummy state
    Control ax_,ay_,az_;
    // AlgebraicState pitch;
    Control s  ;  // slack variable
    // Parameter tx,ty,tz;

    DifferentialEquation model;
    Grid my_grid_( t_start,t_end,time_horizon_ );

    float eps = 0.00001;
    // define the model
    model << dot(px_) == vx_;
    model << dot(py_) == vy_;
    model << dot(pz_) == vz_;
    model << dot(vx_) == ax_;
    model << dot(vy_) == ay_;
    model << dot(vz_) == az_;
    

    OCP ocp(my_grid_);// = new OCP( my_grid_); // possibility to set non equidistant time-horizon of the problem
    ocp.subjectTo(model);

    DVector target_x(my_grid_.getNumPoints());
    DVector target_y(my_grid_.getNumPoints());
    DVector target_z(my_grid_.getNumPoints());
    // // //set target trajectory
    for(uint i=0; i<time_horizon_; i++){
        target_x(i)=_target_trajectory[i].pose.pose.position.x;
        target_y(i)=_target_trajectory[i].pose.pose.position.y;
        target_z(i)=_target_trajectory[i].pose.pose.position.z;
    }

    ocp.subjectTo(  -MAX_ACC <= ax_ <=  MAX_ACC   );  
    ocp.subjectTo(  -MAX_ACC <= ay_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_ACC <= az_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_VEL_XY <= vx_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_XY <= vy_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_Z <= vz_ <= MAX_VEL_Z   );
    ocp.subjectTo(  Z_RELATIVE_TARGET_DRONE <= pz_+s-target_z); 
    ocp.subjectTo(s>=0);

    int closest_point = minDistance(_uavs_pose[_drone_id]);


    if(first_time_solving){
        ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).state.pose.x);
        ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).state.pose.y);
        ocp.subjectTo( AT_START, pz_ == _uavs_pose.at(_drone_id).state.pose.z);
        ocp.subjectTo( AT_START, ax_ == 0.0);
        ocp.subjectTo( AT_START, ay_ == 0.0);
        ocp.subjectTo( AT_START, az_ == 0.0);
    }else{     
        ocp.subjectTo( AT_START, px_ == solution_[(closest_point)+offset_].pose.x);
        ocp.subjectTo( AT_START, py_ == solution_[(closest_point)+offset_].pose.y);
        ocp.subjectTo( AT_START, pz_ == solution_[(closest_point)+offset_].pose.z);
        ocp.subjectTo( AT_START, vx_ == solution_[(closest_point)+offset_].velocity.x);
        ocp.subjectTo( AT_START, vy_ == solution_[(closest_point)+offset_].velocity.y);
        ocp.subjectTo( AT_START, vz_ == solution_[(closest_point)+offset_].velocity.z);
        ocp.subjectTo( AT_START, ax_ == solution_[(closest_point)+offset_].acc.x);
        ocp.subjectTo( AT_START, ay_ == solution_[(closest_point)+offset_].acc.y);
        ocp.subjectTo( AT_START, az_ == solution_[(closest_point)+offset_].acc.z);
    }

    //ocp.subjectTo( s >= 0 ); slack variable

    // Define objectives
    Function h_1;

    h_1 << px_;
    h_1 << py_;


    DMatrix S_1(2,2);
    DVector r_1(2);

    S_1.setIdentity();
	  S_1(0,0) = W_PX_N;
	  S_1(1,1) = W_PY_N;


    r_1(0) = _desired_odometry.pose.pose.position.x;
    r_1(1) = _desired_odometry.pose.pose.position.y;
    // r_1(2) = _desired_odometry.twist.twist.linear.x;
    // r_1(3) = _desired_odometry.twist.twist.linear.y;
    // r_1(4) = _desired_odometry.twist.twist.linear.y;
    // r_1(5) = _desired_odometry.twist.twist.linear.z;
    //use target_x and targety_ptr
    ocp.minimizeLagrangeTerm(pow((pz_-_target_trajectory[0].pose.pose.position.z)/sqrt(
                                                                        pow(px_-_target_trajectory[0].pose.pose.position.x,2)+
                                                                        pow(py_-_target_trajectory[0].pose.pose.position.y,2)
                                                                        +eps)-CAMERA_PITCH
                                ,2));
    ocp.minimizeLSQEndTerm( S_1, h_1, r_1 );

    Function h;

    h << ax_;
    h << ay_;
    h << az_;
    h << s;

    DMatrix S(4,4);
    DVector r(4);

    S.setIdentity();
	  S(0,0) = W_AX;
	  S(1,1) = W_AY;
	  S(2,2) = W_AZ;
    S(3,3) = W_SLACK;

    r(0) = 0.0;
    r(1) = 0.0;
    r(2) = 0.0;
    r(3) = 0.0;
    // r(3) = 3;

    ocp.minimizeLSQ( S, h, r );

    OptimizationAlgorithm solver(ocp);

    ////////////////// INITIALIZATION //////////////////////////////////
    VariablesGrid state_init(6,my_grid_), control_init(4,my_grid_);
   
    for(uint i=0; i<time_horizon_; i++){
        control_init(i,0)= initial_guess_[i].acc.x;
        control_init(i,1)= initial_guess_[i].acc.y;
        control_init(i,2)= initial_guess_[i].acc.z;
        control_init(i,3)=0.0; //slack
        state_init(i,0)= initial_guess_[i].pose.x;
        state_init(i,1)= initial_guess_[i].pose.y;
        state_init(i,2)= initial_guess_[i].pose.z;
        state_init(i,3)= initial_guess_[i].velocity.x;
        state_init(i,4)= initial_guess_[i].velocity.y;
        state_init(i,5)= initial_guess_[i].velocity.z;
       // control(i,3) = _initial_guess["pitch"][i];
    //    inter_state_init(i,0) = 0.2;
    }

    solver.initializeDifferentialStates( state_init );
    solver.initializeControls          ( control_init );
    // solver.initializeAlgebraicStates(inter_state_init);

    //solver.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver.set( INTEGRATOR_TOLERANCE , 1e-8            );
    //solver.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver.set( KKT_TOLERANCE        , 1e-3            );
    // solver.set( MAX_NUM_ITERATIONS        , 5  );
    solver.set( MAX_TIME        , 1.0  );

    // call the solver
    solver_success_ = solver.solve();
    // get solution

    getResults(time_initial_position, solver, first_time_solving, closest_point);

    px_.clearStaticCounters();
    py_.clearStaticCounters();
    pz_.clearStaticCounters();
    vx_.clearStaticCounters();
    vy_.clearStaticCounters();
    vz_.clearStaticCounters();
    ax_.clearStaticCounters();
    ay_.clearStaticCounters();
    az_.clearStaticCounters();
    s.clearStaticCounters();
    // pitch.clearStaticCounters();

    return solver_success_;  
 }


 bool SolverDurable::getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving, const int _closest_point){
    
    VariablesGrid output_states,output_control;

    solver.getDifferentialStates(output_states);
    solver.getControls          (output_control);

    if(solver_success_ == returnValueType::SUCCESSFUL_RETURN || solver_success_ == returnValueType::RET_MAX_TIME_REACHED){ 
        for(int i=0;i<time_horizon_;i++){

            if(i<offset_  && !first_time_solving){ 
                solution_[i].pose.x=       solution_  [_closest_point+i].pose.x;
                solution_[i].pose.y=       solution_  [_closest_point+i].pose.y;
                solution_[i].pose.z=       solution_  [_closest_point+i].pose.z;
                solution_[i].velocity.x=   solution_[_closest_point+i].velocity.x;
                solution_[i].velocity.y=   solution_[_closest_point+i].velocity.y;
                solution_[i].velocity.z=   solution_[_closest_point+i].velocity.z;
                solution_[i].acc.x=      solution_[_closest_point+i].acc.x;
                solution_[i].acc.y=      solution_[_closest_point+i].acc.y;
                solution_[i].acc.z=      solution_[_closest_point+i].acc.z;
            }else{
                solution_[i].pose.x=output_states(i-offset_*(int)!first_time_solving,0);
                solution_[i].pose.y=output_states(i-offset_*(int)!first_time_solving,1);
                solution_[i].pose.z=output_states(i-offset_*(int)!first_time_solving,2);
                solution_[i].velocity.x=output_states(i-offset_*(int)!first_time_solving,3);
                solution_[i].velocity.y=output_states(i-offset_*(int)!first_time_solving,4);
                solution_[i].velocity.z=output_states(i-offset_*(int)!first_time_solving,5);
                solution_[i].acc.x=output_control(i-offset_*(int)!first_time_solving,0);
                solution_[i].acc.y=output_control(i-offset_*(int)!first_time_solving,1);
                solution_[i].acc.z=output_control(i-offset_*(int)!first_time_solving,2);
                // csv<<output_control(i,3)<<std::endl;
            }
        }
    }
    return true;
 }