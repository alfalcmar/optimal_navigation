#include<solver_acado.h>


NumericalSolver::ACADOSolver::ACADOSolver(const float solving_rate, const int time_horizon, const std::shared_ptr<State[]> &initial_guess, 
                                         const std::shared_ptr<safe_corridor_generator::SafeCorridorGenerator> _safe_corridor_generator_ptr) : 
                                        Solver(solving_rate, time_horizon, initial_guess, _safe_corridor_generator_ptr){
    

}

std::vector<double> NumericalSolver::ACADOSolver::orientation(const std::vector<nav_msgs::Odometry> &_target_trajectory, float actual_heading, const std::unique_ptr<State[]> &_solution){
  // // DifferentialState heading, v_heading;
  DifferentialState heading, v_heading;
  Control a_heading;

  float a_limit     = 0.3;
  float v_limit     = 0.5;

  DifferentialEquation model;
  model << dot(heading) == v_heading;
  model << dot(v_heading) == a_heading;
  
  Grid my_grid( t_start,t_end,time_horizon_ );
  OCP ocp(my_grid);  
  ocp.subjectTo(model);
  ocp.subjectTo(-a_limit <= a_heading <= a_limit);
  ocp.subjectTo(-v_limit <= v_heading <= v_limit);

  VariablesGrid reference_trajectory(2, my_grid); // 2 for pitch
  DVector       reference_point(2); // 2 for pitch
  reference_trajectory.setVector(0, reference_point);  // TODO: check indexing
  Eigen::Vector3f q_camera_target;
  for (int k = 0; k < time_horizon_; k++) {
    q_camera_target = Eigen::Vector3f(_target_trajectory[k].pose.pose.position.x, _target_trajectory[k].pose.pose.position.y, _target_trajectory[k].pose.pose.position.z) - 
                        Eigen::Vector3f(_solution[k].pose.x, _solution[k].pose.y, _solution[k].pose.z);
    reference_point(0) = atan2(q_camera_target[1], q_camera_target[0]);
    reference_point(1) = 0;
    reference_trajectory.setVector(k, reference_point);  // TODO: check indexing
  }

         
    ocp.subjectTo( AT_START, heading == actual_heading);
    ocp.subjectTo( AT_START, v_heading == 0.0);
    ocp.subjectTo( AT_START, a_heading == 0.0);
    

   Function rf;

   rf << heading << a_heading;

   DMatrix S(2, 2);

   S.setIdentity();
   S(0, 0) = 1.0;
   S(1, 1) = 1.0;

   ocp.minimizeLSQ(S, rf, reference_trajectory);

 
   OptimizationAlgorithm solver_(ocp);

  // // reference_trajectory.print();
  // // call the solver
  returnValue value = solver_.solve();
  // // get solution
  VariablesGrid output_states, output_control;

  solver_.getDifferentialStates(output_states);
  solver_.getControls(output_control);
  // ROS_INFO("[Acado]: Output states: ");
//   output_states.print();

  std::cout<<"heading"<<std::endl;
  std::vector<double> yaw;
  for (uint i = 0; i < time_horizon_; i++) {
    yaw.push_back(output_states(i, 0));
  }
  
  heading.clearStaticCounters();
  v_heading.clearStaticCounters();
  a_heading.clearStaticCounters();
  int success_value = value;
//   ROS_INFO("[Acado]: Acado angular optimization took %.3f, success = %d ", (ros::Time::now() - start).toSec(), success_value);
  return yaw;
}


int NumericalSolver::ACADOSolver::mpc(ros::Publisher &pub_path_, ros::Publisher &pub_corridor_polyhedrons_, const std::map<int,UavState> &_uavs_pose, const int _drone_id){
    DifferentialState px_,py_,pz_,vx_,vy_,vz_;

    Control ax_,ay_,az_;

    DifferentialEquation model;
    Grid my_grid( t_start,t_end,time_horizon_ );

    // define the model
    model << dot(px_) == vx_;
    model << dot(py_) == vy_;
    model << dot(pz_) == vz_;
    model << dot(vx_) == ax_;
    model << dot(vy_) == ay_;
    model << dot(vz_) == az_;
    

    OCP ocp(my_grid);
    ocp.subjectTo(model);


    ocp.subjectTo(  -MAX_ACC <= ax_ <= MAX_ACC   );  
    ocp.subjectTo(  -MAX_ACC <= ay_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_ACC <= az_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_VEL_XY <= vx_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_XY <= vy_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_Z <= vz_ <= MAX_VEL_Z   );


    ocp.subjectTo( AT_START, px_ == solution_[0].pose.x);
    ocp.subjectTo( AT_START, py_ == solution_[0].pose.y);
    ocp.subjectTo( AT_START, pz_ == solution_[0].pose.z);
    ocp.subjectTo( AT_START, vx_ == solution_[0].velocity.x);
    ocp.subjectTo( AT_START, vy_ == solution_[0].velocity.y);
    ocp.subjectTo( AT_START, vz_ == solution_[0].velocity.z);
    ocp.subjectTo( AT_START, ax_ == solution_[0].acc.x);
    ocp.subjectTo( AT_START, ay_ == solution_[0].acc.y);
    ocp.subjectTo( AT_START, az_ == solution_[0].acc.z);

    ////////// polyhedrons/////////////////
    geometry_msgs::PoseStamped pose_aux;
    nav_msgs::PathPtr                       path_ref(new nav_msgs::Path);

    for(int i=0; i<time_horizon_;i++){ // solution to nav msgs
        pose_aux.pose.position.x = solution_[i].pose.x;
        pose_aux.pose.position.y = solution_[i].pose.y;
        pose_aux.pose.position.z = solution_[i].pose.z;

        path_ref->poses.push_back(pose_aux);
    }


    vec_Vec3f path_ref_vector;

    for(int i=0; i<path_ref->poses.size();i++){ // nav_msgs to eigen vector
        path_ref_vector.push_back(Vec3f(path_ref->poses[i].pose.position.x,path_ref->poses[i].pose.position.y, path_ref->poses[i].pose.position.z));
        std::cout<<"x: "<<path_ref->poses[i].pose.position.x<<" y: "<<path_ref->poses[i].pose.position.y<<" z: "<<path_ref->poses[i].pose.position.z<<std::endl;
    }
    
    // if the last point is occupied
    if(safe_corridor_generator_->isPointOccupied(path_ref_vector.back())){
        Vec3f last_point = path_ref_vector.back();
        const Vec3f first_point = path_ref_vector[0];
        const Vec3f point_dir = (last_point-first_point)/(last_point-first_point).norm();
        while(safe_corridor_generator_->isPointOccupied(last_point)){
            last_point = last_point+point_dir;
        }
        path_ref_vector.back() = last_point;
        path_ref->poses.back().pose.position.x = last_point(0);
        path_ref->poses.back().pose.position.y = last_point(1);
        path_ref->poses.back().pose.position.z = last_point(2);
    }
    
    // add follower to the map
    geometry_msgs::Point aux_point;
    const double raidus = 0.5;
    const int n_points = 100;
    for (auto it=_uavs_pose.begin(); it!=_uavs_pose.end(); ++it){
        if(it->first!=_drone_id){
            aux_point.x = it->second.state.pose.x;
            aux_point.y = it->second.state.pose.y;
            aux_point.z = it->second.state.pose.z;
            std::cout<<"adding follower to map with pose: "<<"x:  "<<aux_point.x<<" y: "<<aux_point.y<<" z: "<<aux_point.z<<std::endl;
            safe_corridor_generator_->addPositionOfRobotToPclMap(aux_point, raidus, 5.0, n_points);

        }
    }  

    vec_E<Polyhedron<3>> polyhedron_vector = safe_corridor_generator_->getSafeCorridorPolyhedronVector(path_ref); // get polyhedrons

    // get JPS path along which the polyhedrons were generated - needed to generation of correct constraints
    nav_msgs::PathPtr collision_free_path = safe_corridor_generator_->getLastPath();

    vec_Vec3f collision_free_path_vector;

    for(int i = 0; i <collision_free_path->poses.size(); i++){ // nav_msgs to eigen vector
        collision_free_path_vector.push_back(Vec3f(collision_free_path->poses[i].pose.position.x,collision_free_path->poses[i].pose.position.y, collision_free_path->poses[i].pose.position.z));
        std::cout<<"x: "<<collision_free_path->poses[i].pose.position.x<<" y: "<<collision_free_path->poses[i].pose.position.y<<" z: "<<collision_free_path->poses[i].pose.position.z<<std::endl;
    }

    polyhedronsToACADO(ocp, polyhedron_vector, collision_free_path_vector, px_, py_, pz_ ); // polyhedrons to acado

    // safe_corridor_generator_->publishLastPath(pub_path_);
  
    // safe_corridor_generator_->publishCorridor(pub_corridor_polyhedrons_);

    ////////////////////////////////////////

    // setup reference trajectory
    ROS_INFO("[Acado]: Set reference trajectory");
    VariablesGrid reference_trajectory(3, t_start,t_end,time_horizon_ );
    DVector       reference_point(3);
    for (int k = 0; k < time_horizon_; k++) { // previous solution to reference
      reference_point(0) = solution_[k].pose.x;
      reference_point(1) = solution_[k].pose.y;
      reference_point(2) = solution_[k].pose.z;
      reference_trajectory.setVector(k, reference_point);  // TODO: check indexing
    }


  // DEFINE LSQ function to minimize diff from desired trajectory
  Function rf;

  rf << px_;
  rf << py_;
  rf << pz_;

  DMatrix S(3, 3);

  S.setIdentity();
  S(0, 0) = 0.1;
  S(1, 1) = 0.1;
  S(2, 2) = 0.1;

  ROS_INFO("[%s]: Reference size: points = %u, cols = %u, rows = %u", ros::this_node::getName().c_str(), reference_trajectory.getNumPoints(),
           reference_trajectory.getNumCols(), reference_trajectory.getNumRows());
  ocp.minimizeLSQ(S, rf, reference_trajectory);

  // DEFINE LSQ function to minimize accelerations
  Function rf_a;

  rf_a << ax_;
  rf_a << ay_;
  rf_a << az_;

  DMatrix S_a(3, 3);
  DVector r_a(3);

  r_a.setZero();

  S_a.setIdentity();
  S_a(0, 0) = 1.0;
  S_a(1, 1) = 1.0;
  S_a(2, 2) = 1.0;

  ocp.minimizeLSQ(S, rf_a, r_a);

  ROS_INFO("[Acado]: Objective functions defined");

  OptimizationAlgorithm solver(ocp);  

    //solver.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver.set( INTEGRATOR_TOLERANCE , 1e-3            ); //1e-8
    solver.set( KKT_TOLERANCE        , 1e-1            ); // 1e-3
    // solver.set( MAX_NUM_ITERATIONS        , 5  );
    // solver.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    

    solver.set( MAX_TIME        , 2.0  );
    // call the solver
    bool solver_success = solver.solve();
    // get solution
    VariablesGrid output_states,output_control;


    solver.getDifferentialStates(output_states);
    solver.getControls          (output_control);

    for(int k =0; k<time_horizon_;k++){
        solution_[k].pose.x = output_states(k,0);
        solution_[k].pose.y = output_states(k,1);
        solution_[k].pose.z = output_states(k,2);
        solution_[k].velocity.x = output_states(k,3);
        solution_[k].velocity.y = output_states(k,4);
        solution_[k].velocity.z = output_states(k,5);
        solution_[k].acc.x = output_control(k,0);
        solution_[k].acc.y = output_control(k,1);
        solution_[k].acc.z = output_control(k,2);
    }

    px_.clearStaticCounters();
    py_.clearStaticCounters();
    pz_.clearStaticCounters();
    vx_.clearStaticCounters();
    vy_.clearStaticCounters();
    vz_.clearStaticCounters();
    ax_.clearStaticCounters();
    ay_.clearStaticCounters();
    az_.clearStaticCounters();

    if(solver_success==returnValueType::SUCCESSFUL_RETURN ||solver_success==returnValueType::RET_MAX_TIME_REACHED ) ROS_INFO("MPC was successful");
    else ROS_INFO("MPC cannot solve");
    return solver_success;
}

int NumericalSolver::ACADOSolver::solverFunction( nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,UavState> &_uavs_pose, ros::Publisher &pub_path_, ros::Publisher &pub_corridor_polyhedrons_, const int discarted_points, bool first_time_solving,  int _drone_id, bool _target /*false*/,bool _multi/*false*/){
    ROS_INFO("[%s]: Starting solver function.", ros::this_node::getName().c_str());
    DifferentialState px_,py_,pz_,vx_,vy_,vz_;
    //DifferentialState   dummy;  // dummy state
    Control ax_,ay_,az_;
    // AlgebraicState pitch;
    /* Control s  ;  // slack variable */
    // Parameter tx,ty,tz;

    float time_initial_position = 1/solving_rate_;

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
    ROS_INFO("[%s]: Solver function set target trajectory. time_horizon = %d, target_size = %lu, target_trajectory = %lu", ros::this_node::getName().c_str(), time_horizon_, target_x.size(), _target_trajectory.size());
    for(uint i=0; i<time_horizon_; i++){
        target_x(i)=_target_trajectory[i].pose.pose.position.x;
        target_y(i)=_target_trajectory[i].pose.pose.position.y;
        target_z(i)=_target_trajectory[i].pose.pose.position.z;
    }

    ROS_INFO("[%s]: Solver function limits: max_acc = %.2f, max_v_xy = %.2f, max_v_z = %.2f", ros::this_node::getName().c_str(), MAX_ACC, MAX_VEL_XY, MAX_VEL_Z);
    ocp.subjectTo(  -MAX_ACC <= ax_ <=  MAX_ACC   );  
    ocp.subjectTo(  -MAX_ACC <= ay_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_ACC <= az_ <= MAX_ACC   );
    ocp.subjectTo(  -MAX_VEL_XY <= vx_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_XY <= vy_ <= MAX_VEL_XY   );
    ocp.subjectTo(  -MAX_VEL_Z <= vz_ <= MAX_VEL_Z   );
    /* ocp.subjectTo(  Z_RELATIVE_TARGET_DRONE <= pz_+s-target_z); */ 
    /* ocp.subjectTo(s>=0); */


    ROS_INFO("[%s]: Solver function 2.", ros::this_node::getName().c_str());
    if(first_time_solving){
        ROS_INFO("[%s]: Solver function first time solving. dron_id = %d, uavs_poses size = %lu, dorn position = [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), _drone_id, _uavs_pose.size(), _uavs_pose.at(_drone_id).state.pose.x, _uavs_pose.at(_drone_id).state.pose.y, _uavs_pose.at(_drone_id).state.pose.z);
        ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).state.pose.x);
        ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).state.pose.y);
        ocp.subjectTo( AT_START, pz_ == _uavs_pose.at(_drone_id).state.pose.z);
        ocp.subjectTo( AT_START, vx_ == 0.0);
        ocp.subjectTo( AT_START, vy_ == 0.0);
        ocp.subjectTo( AT_START, vz_ == 0.0);
        ocp.subjectTo( AT_START, ax_ == 0.0);
        ocp.subjectTo( AT_START, ay_ == 0.0);
        ocp.subjectTo( AT_START, az_ == 0.0);
    }else{     
        // FIXME: I think that it does not make sense to put constraints on current acceleration if you penalize only its absolute value and not diff. It is a control input, so by these constraints you directly decide what will be the complete state in iteration AT_START + 1 
        ROS_INFO("[%s]: Solver function not first time solving. time_initial_position/step_size = %d, offset_ = %d, discarded_points=%d ", ros::this_node::getName().c_str(), (time_initial_position/step_size), offset_, discarted_points);
        ocp.subjectTo( AT_START, px_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].pose.x);
        ocp.subjectTo( AT_START, py_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].pose.y);
        ocp.subjectTo( AT_START, pz_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].pose.z);
        ocp.subjectTo( AT_START, vx_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].velocity.x);
        ocp.subjectTo( AT_START, vy_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].velocity.y);
        ocp.subjectTo( AT_START, vz_ == solution_[(time_initial_position/step_size)+offset_+discarted_points].velocity.z);
        /* ocp.subjectTo( AT_START, ax_ == solution_[(time_initial_position/step_size)+offset_].acc.x); */
        /* ocp.subjectTo( AT_START, ay_ == solution_[(time_initial_position/step_size)+offset_].acc.y); */
        /* ocp.subjectTo( AT_START, az_ == solution_[(time_initial_position/step_size)+offset_].acc.z); */
    }

    ROS_INFO("[%s]: Solver function 2.5", ros::this_node::getName().c_str());
    // Define objectives
    /* Function h_1; */

    /* h_1 << px_; */
    /* h_1 << py_; */


    /* DMatrix S_1(2,2); */
    /* DVector r_1(2); */

    /* S_1.setIdentity(); */
	  /* S_1(0,0) = W_PX_N; */
	  /* S_1(1,1) = W_PY_N; */


    /* r_1(0) = _desired_odometry.pose.pose.position.x; */
    /* r_1(1) = _desired_odometry.pose.pose.position.y; */

    ROS_INFO("[%s]: Solver objective 1: ", ros::this_node::getName().c_str());
    //use target_x and targety_ptr
    /* ocp.minimizeLagrangeTerm(W_CIN*pow((pz_-target_z)/sqrt( */
    /*                                                pow(px_-target_x,2)+ */
    /*                                                pow(py_-target_y,2) */
    /*                                                +eps)-CAMERA_PITCH,2)); */
    /* ocp.minimizeLSQEndTerm( S_1, h_1, r_1 ); */

  // setup reference trajectory
  ROS_INFO("[Acado]: Set reference trajectory");
  VariablesGrid reference_trajectory(3, t_start, t_end, time_horizon_);
  DVector       reference_point(3);
  for (int k = 0; k < time_horizon_; k++) {
    reference_point(0) = _uavs_pose.at(_drone_id).state.pose.x + 5.0;
    reference_point(1) = _uavs_pose.at(_drone_id).state.pose.y;
    reference_point(2) = 2.0;
    reference_trajectory.setVector(k, reference_point);  // TODO: check indexing
  }

  // DEFINE LSQ function to minimize diff from desired trajectory
  Function rf;

  rf << px_;
  rf << py_;
  rf << pz_;

  DMatrix S_r(3, 3);

  S_r.setIdentity();
  S_r(0, 0) = 1.0;
  S_r(1, 1) = 1.0;
  S_r(2, 2) = 1.0;

  /* ROS_INFO("[%s]: Reference size: points = %u, cols = %u, rows = %u", ros::this_node::getName().c_str(), reference_trajectory.getNumPoints(), */
  /*          reference_trajectory.getNumCols(), reference_trajectory.getNumRows()); */
  ocp.minimizeLSQ(S_r, rf, reference_trajectory);


    Function h;

    h << ax_;
    h << ay_;
    h << az_;
    /* h << s; */

    DMatrix S(3,3);
    DVector r(3);

    ROS_INFO("[%s]: Solver objective 1: wax = %.2f, way = %.2f, waz = %.2f", ros::this_node::getName().c_str());

    S.setIdentity();
    S(0,0) = W_AX;
    S(1,1) = W_AY;
    S(2,2) = W_AZ;
    /* S(3,3) = W_SLACK; */

    r.setZero();
    r(0) = 0.0;
    r(1) = 0.0;
    r(2) = 0.0;
    /* r(3) = 0.0; */
    // r(3) = 3;

    ocp.minimizeLSQ( S, h, r );

    OptimizationAlgorithm solver(ocp);

    ////////////////// INITIALIZATION //////////////////////////////////
    VariablesGrid state_init(6,my_grid_), control_init(4,my_grid_);
   
    for(uint i=0; i<time_horizon_; i++){
        control_init(i,0)= 0.0;
        control_init(i,1)= 0.0;
        control_init(i,2)= 0.0;
        /* control_init(i,3)=0.0; //slack */
        state_init(i,0)= _uavs_pose.at(_drone_id).state.pose.x;
        state_init(i,1)= _uavs_pose.at(_drone_id).state.pose.y;
        state_init(i,2)= _uavs_pose.at(_drone_id).state.pose.z;
        state_init(i,3)= 0.0;
        state_init(i,4)= 0.0;
        state_init(i,5)= 0.0;
       // control(i,3) = _initial_guess["pitch"][i];
    //    inter_state_init(i,0) = 0.2;
    }

    solver.initializeDifferentialStates( state_init );
    solver.initializeControls          ( control_init );
    // solver.initializeAlgebraicStates(inter_state_init);

  px_.clearStaticCounters();
  py_.clearStaticCounters();
  pz_.clearStaticCounters();
  vx_.clearStaticCounters();
  vy_.clearStaticCounters();
  vz_.clearStaticCounters();
  ax_.clearStaticCounters();
  ay_.clearStaticCounters();
  az_.clearStaticCounters();

    //solver.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver.set( INTEGRATOR_TOLERANCE , 1e-1            ); //1e-8
    //solver.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver.set( KKT_TOLERANCE        , 1e-1            ); // 1e-3
    /* solver.set( MAX_NUM_ITERATIONS        , 5  ); */
    solver.set( MAX_TIME        , 1.0  );
    /* solver.set( PRINTLEVEL        , 10.0); */

    // call the solver
    ROS_INFO("[%s]: Calling solver.", ros::this_node::getName().c_str());
    solver_success_ = solver.solve();
    // get solution

    ROS_INFO("[%s]: Solver function 3.", ros::this_node::getName().c_str());
    getResults(time_initial_position, solver, first_time_solving, discarted_points);

    px_.clearStaticCounters();
    py_.clearStaticCounters();
    pz_.clearStaticCounters();
    vx_.clearStaticCounters();
    vy_.clearStaticCounters();
    vz_.clearStaticCounters();
    ax_.clearStaticCounters();
    ay_.clearStaticCounters();
    az_.clearStaticCounters();
    /* s.clearStaticCounters(); */
    // pitch.clearStaticCounters();
    bool mpc_return = false; 
    if (solver_success_==returnValueType::SUCCESSFUL_RETURN || solver_success_==returnValueType::RET_MAX_TIME_REACHED) {
       ROS_INFO("solving mpc");
       mpc_return =   mpc(pub_path_,pub_corridor_polyhedrons_, _uavs_pose, _drone_id); //TODO: think about how to manage the return of mpc
       if(mpc_return==returnValueType::SUCCESSFUL_RETURN){
           return solver_success_;
       }else{
           return mpc_return;
       }
    }else{
        return solver_success_;  
    }

 }

void NumericalSolver::ACADOSolver::polyhedronsToACADO(OCP &_ocp, const vec_E<Polyhedron<3>> &_vector_of_polyhedrons, const vec_Vec3f &_initial_path, DifferentialState &_px, DifferentialState &_py, DifferentialState &_pz){
   
   // Convert to inequality constraints Ax < b
   // Taken from decomp test node
   ROS_INFO("[Acado]: polyhedrons to acado ");

    for (size_t i = 0; i < _initial_path.size() - 1; i++) {

        ROS_INFO("[Acado]: polyhedrons to acado - iter i = %lu, initial path size = %lu, polyhedrons size =%lu ", i, _initial_path.size(),_vector_of_polyhedrons.size());
        const auto         pt_inside = _initial_path[i+1];

        LinearConstraint3D cs(pt_inside, _vector_of_polyhedrons[i].hyperplanes());
        for(size_t k = 0; k<cs.b().size(); k++){ //each polyhedron i is subject to k constraints
            ROS_INFO("[Acado]: polyhedrons to acado - iter k = %lu ", k);
            if(!std::isnan(cs.A()(k, 0)) && !std::isnan(cs.A()(k, 1)) && !std::isnan(cs.A()(k, 2)) && !std::isnan(cs.b()[k])) {
                _ocp.subjectTo(i+1, cs.A()(k, 0) * _px + cs.A()(k, 1) * _py + cs.A()(k, 2) * _pz <= cs.b()[k]);
            } else {
                ROS_ERROR("[Acado]: NaNs detected in polyhedrons ");
            }
                ROS_INFO("[Acado]: Adding constraint: %.2f * px + %.2f * py + %.2f * pz <= %.2f", cs.A()(k, 0), cs.A()(k, 1), cs.A()(k, 2), cs.b()[k]);
                ROS_INFO("[Acado]: Point: [%.2f, %.2f, %.2f], result = %.2f", pt_inside(0), pt_inside(1), pt_inside(2), cs.A()(k, 0) * pt_inside(0) + cs.A()(k, 1) * pt_inside(1) + cs.A()(k, 2) * pt_inside(2));
        }
    }
}


 bool NumericalSolver::ACADOSolver::logACADOvars(){
    // define the solver
    // LogRecord logRecord(LOG_AT_EACH_ITERATION);
    // logRecord << LOG_NUM_NLP_ITERATIONS;
    // logRecord << LOG_KKT_TOLERANCE;
    // //logRecord << LOG_OBJECTIVE_FUNCTION;
    // logRecord << LOG_MERIT_FUNCTION_VALUE;
    // logRecord << LOG_LINESEARCH_STEPLENGTH;
    // //logRecord << LOG_ALGREBRAIC_STATES;
    // logRecord << LOG_CONTROLS;
    // logRecord << LOG_DISTURBANCES;
    // logRecord << LOG_INTERMEDIATE_STATES;
    // logRecord << LOG_DIFFERENTIAL_STATES;
    return true;
 }

 bool NumericalSolver::ACADOSolver::testPolyhedronConstraints(const std::vector<geometry_msgs::PoseStamped> &_path, const vec_E<Polyhedron<3>> &_polyhedrons){


     return true;
 }

 bool NumericalSolver::ACADOSolver::getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving, const int discarted_points){
    
    VariablesGrid output_states,output_control;

    solver.getDifferentialStates(output_states);
    solver.getControls          (output_control);

    if(solver_success_ == returnValueType::SUCCESSFUL_RETURN || solver_success_ == returnValueType::RET_MAX_TIME_REACHED){ 
        for(int i=0;i<time_horizon_;i++){

            if(i<offset_  && !first_time_solving){ 
                solution_[i].pose.x=       solution_  [(time_initial_position/step_size)+i+discarted_points].pose.x;
                solution_[i].pose.y=       solution_  [(time_initial_position/step_size)+i+discarted_points].pose.y;
                solution_[i].pose.z=       solution_  [(time_initial_position/step_size)+i+discarted_points].pose.z;
                solution_[i].velocity.x=   solution_[(time_initial_position/step_size)+i+discarted_points].velocity.x;
                solution_[i].velocity.y=   solution_[(time_initial_position/step_size)+i+discarted_points].velocity.y;
                solution_[i].velocity.z=   solution_[(time_initial_position/step_size)+i+discarted_points].velocity.z;
                solution_[i].acc.x=      solution_[(time_initial_position/step_size)+i+discarted_points].acc.x;
                solution_[i].acc.y=      solution_[(time_initial_position/step_size)+i+discarted_points].acc.y;
                solution_[i].acc.z=      solution_[(time_initial_position/step_size)+i+discarted_points].acc.z;
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
            }
        }
    }
    return true;
 }

nav_msgs::Path NumericalSolver::ACADOSolver::calculatePath(const Vec2f &start_pose, const Vec2f &final_pose, const State &_uav_state, const std::vector<nav_msgs::Odometry> &_target_trajectory){

    float segment_dist = ((final_pose-start_pose).norm())/time_horizon_;

    Vec2f segment_uni = (final_pose-start_pose)/(final_pose-start_pose).norm();
    geometry_msgs::PoseStamped              ps;
    std::vector<geometry_msgs::PoseStamped> ps_vector;
    
    for (int k=0; k<time_horizon_;k++){
        ps.pose.position.x              = start_pose(0)+segment_uni(0)*k*segment_dist;
        ps.pose.position.y              = start_pose(1)+segment_uni(1)*k*segment_dist;
        ps.pose.position.z              = CAMERA_PITCH*(sqrt(pow(ps.pose.position.x-_target_trajectory[k].pose.pose.position.x,2)+pow(ps.pose.position.y-_target_trajectory[k].pose.pose.position.y,2)))+_target_trajectory[k].pose.pose.position.z;
        if( Z_RELATIVE_TARGET_DRONE >ps.pose.position.z-_target_trajectory.back().pose.pose.position.z){
           ps.pose.position.z =   Z_RELATIVE_TARGET_DRONE+_target_trajectory.back().pose.pose.position.z;
        }
        ps_vector.push_back(ps);
    }
    // Vec3f last_point(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z);
    // int k = 1;
    // while(safe_corridor_generator->isPointOccupied(last_point)){
    //     ps_vector.back().pose.position.x              = last_point(0)+segment_uni(0)*k*segment_dist;
    //     ps_vector.back().pose.position.y              = last_point(1)+segment_uni(1)*k*segment_dist;
    //     ps_vector.back().pose.position.z              = CAMERA_PITCH*(sqrt(pow(ps.pose.position.x-_target_trajectory[k].pose.pose.position.x,2)+pow(ps.pose.position.y-_target_trajectory[k].pose.pose.position.y,2)))+_target_trajectory[k].pose.pose.position.z;
    //     k= k+1;
    // }
    nav_msgs::Path path;
    path.poses = ps_vector;

    return path;
}   
