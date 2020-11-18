#include<numericalSolver.h>    

NumericalSolver::Solver::Solver(const float solving_rate, const int time_horizon) : solving_rate_(solving_rate),
                                                                        time_horizon_(time_horizon),
                                                                        x_ptr_(new double[time_horizon]{0.0}),
                                                                        y_ptr_(new double[time_horizon]{0.0}),
                                                                        z_ptr_(new double[time_horizon]{0.0}),
                                                                        vx_ptr_(new double[time_horizon]{0.0}),
                                                                        vy_ptr_(new double[time_horizon]{0.0}),
                                                                        vz_ptr_(new double[time_horizon]{0.0}),
                                                                        ax_ptr_(new double[time_horizon]{0.0}),
                                                                        ay_ptr_(new double[time_horizon]{0.0}),
                                                                        az_ptr_(new double[time_horizon]{0.0})
{



}

int NumericalSolver::Solver::solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){

}

NumericalSolver::ACADOSolver::ACADOSolver(const float solving_rate, const int time_horizon) : Solver(solving_rate, time_horizon){
    

}



int NumericalSolver::ACADOSolver::solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess, nav_msgs::Odometry &_desired_odometry, const std::vector<float> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, float time_initial_position, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){
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



    if(first_time_solving){
        ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).pose.pose.position.x);
        ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).pose.pose.position.y);
        ocp.subjectTo( AT_START, pz_ == _uavs_pose.at(_drone_id).pose.pose.position.z);
        ocp.subjectTo( AT_START, ax_ == 0.0);
        ocp.subjectTo( AT_START, ay_ == 0.0);
        ocp.subjectTo( AT_START, az_ == 0.0);
    }else{     
        ocp.subjectTo( AT_START, px_ == x_ptr_ [(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, py_ == y_ptr_ [(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, pz_ == z_ptr_ [(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, vx_ == vx_ptr_[(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, vy_ == vy_ptr_[(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, vz_ == vz_ptr_[(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, ax_ == ax_ptr_[(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, ay_ == ay_ptr_[(time_initial_position/step_size)+offset_]);
        ocp.subjectTo( AT_START, az_ == az_ptr_[(time_initial_position/step_size)+offset_]);
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
        control_init(i,0)=_initial_guess["ax"][i];
        control_init(i,1)=_initial_guess["ay"][i];
        control_init(i,2)=_initial_guess["az"][i];
        control_init(i,3)=0.0; //slack
        state_init(i,0)=_initial_guess["px"][i];
        state_init(i,1)=_initial_guess["py"][i];
        state_init(i,2)=_initial_guess["pz"][i];
        state_init(i,3)=_initial_guess["vx"][i];
        state_init(i,4)=_initial_guess["vy"][i];
        state_init(i,5)=_initial_guess["vz"][i];
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

    getResults(time_initial_position, solver, first_time_solving);

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

 bool NumericalSolver::ACADOSolver::getResults(const float time_initial_position, const OptimizationAlgorithm& solver, const bool first_time_solving){
    
    VariablesGrid output_states,output_control;

    solver.getDifferentialStates(output_states);
    solver.getControls          (output_control);

    if(solver_success_ == returnValueType::SUCCESSFUL_RETURN || solver_success_ == returnValueType::RET_MAX_TIME_REACHED){ 
        for(int i=0;i<time_horizon_;i++){

            if(i<offset_  && !first_time_solving){ 
                x_ptr_[i]=x_ptr_  [(time_initial_position/step_size)+i];
                y_ptr_[i]=y_ptr_  [(time_initial_position/step_size)+i];
                z_ptr_[i]=z_ptr_  [(time_initial_position/step_size)+i];
                vx_ptr_[i]=vx_ptr_[(time_initial_position/step_size)+i];
                vy_ptr_[i]=vy_ptr_[(time_initial_position/step_size)+i];
                vz_ptr_[i]=vz_ptr_[(time_initial_position/step_size)+i];
                ax_ptr_[i]=ax_ptr_[(time_initial_position/step_size)+i];
                ay_ptr_[i]=ay_ptr_[(time_initial_position/step_size)+i];
                az_ptr_[i]=az_ptr_[(time_initial_position/step_size)+i];
            }else{
                x_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,0);
                y_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,1);
                z_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,2);
                vx_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,3);
                vy_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,4);
                vz_ptr_[i]=output_states(i-offset_*(int)!first_time_solving,5);
                ax_ptr_[i]=output_control(i-offset_*(int)!first_time_solving,0);
                ay_ptr_[i]=output_control(i-offset_*(int)!first_time_solving,1);
                az_ptr_[i]=output_control(i-offset_*(int)!first_time_solving,2);
                // csv<<output_control(i,3)<<std::endl;
            }
        }
    }
    return true;
 }
