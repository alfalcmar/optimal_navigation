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

    ocp.subjectTo( -1.0 <= ax_ <=  1.0   );  
    ocp.subjectTo(  -1.0 <= ay_ <= 1.0   );
    ocp.subjectTo(  -1.0 <= az_ <= 1.0   );
    // ocp.subjectTo(  -50.0 <= px_ <= 50.0   );
    // ocp.subjectTo(  -50.0 <= py_ <= 50.0   );
    ocp.subjectTo(  Z_RELATIVE_TARGET_DRONE <= pz_+s-target_z); 
    ocp.subjectTo(s>=0);
    ocp.subjectTo(  -1 <= vx_ <= 1   );
    ocp.subjectTo(  -1 <= vy_ <= 1   );
    ocp.subjectTo(  -0.5 <= vz_ <= 0.5   );
    //ocp.subjectTo( -M_PI_4 <=pitch <= M_PI_2); //pitch constraint
    // csv<<"first time: "<<first_time_solving<<std::endl; // check it
    // csv<<"time intial position: "<<time_initial_position<<std::endl;
    // ocp.minimizeLagrangeTerm(ax*ax+ay*ay);  // weight this with the physical cost!!!
    if(first_time_solving){
        ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).pose.pose.position.x);
        ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).pose.pose.position.y);
        ocp.subjectTo( AT_START, pz_ == _uavs_pose.at(_drone_id).pose.pose.position.z);
        ocp.subjectTo( AT_START, ax_ == 0.0);
        ocp.subjectTo( AT_START, ay_ == 0.0);
        ocp.subjectTo( AT_START, az_ == 0.0);
    }else{     
        ocp.subjectTo( AT_START, px_ == x_ptr_[time_initial_position/step_size]);
        ocp.subjectTo( AT_START, py_ == y_ptr_[time_initial_position/step_size]);
        ocp.subjectTo( AT_START, pz_ == z_ptr_[time_initial_position/step_size]);
        ocp.subjectTo( 1, px_ == x_ptr_[time_initial_position/step_size+1]);
        ocp.subjectTo( 1, py_ == y_ptr_[time_initial_position/step_size+1]);
        ocp.subjectTo( 1, pz_ == z_ptr_[time_initial_position/step_size+1]);
        ocp.subjectTo( 2, px_ == x_ptr_[time_initial_position/step_size+2]);
        ocp.subjectTo( 2, py_ == y_ptr_[time_initial_position/step_size+2]);
        ocp.subjectTo( 2, pz_ == z_ptr_[time_initial_position/step_size+2]);
        ocp.subjectTo( 3, px_ == x_ptr_[time_initial_position/step_size+3]);
        ocp.subjectTo( 3, py_ == y_ptr_[time_initial_position/step_size+3]);
        ocp.subjectTo( 3, pz_ == z_ptr_[time_initial_position/step_size+3]);
        ocp.subjectTo( 4, px_ == x_ptr_[time_initial_position/step_size+4]);
        ocp.subjectTo( 4, py_ == y_ptr_[time_initial_position/step_size+4]);
        ocp.subjectTo( 4, pz_ == z_ptr_[time_initial_position/step_size+4]);
        ocp.subjectTo( 5, px_ == x_ptr_[time_initial_position/step_size+5]);
        ocp.subjectTo( 5, py_ == y_ptr_[time_initial_position/step_size+5]);
        ocp.subjectTo( 5, pz_ == z_ptr_[time_initial_position/step_size+5]);
    }

    //ocp.subjectTo( s >= 0 ); slack variable

    // Define objectives
    Function h_1;

    h_1 << px_;
    h_1 << py_;


    DMatrix S_1(2,2);
    DVector r_1(2);

    S_1.setIdentity();
	S_1(0,0) = 0.1;
	S_1(1,1) = 0.1;


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
	S(0,0) = 1;
	S(1,1) = 1;
	S(2,2) = 1.0;
    S(3,3) = 5.0;

    r(0) = 0.0;
    r(1) = 0.0;
    r(2) = 0.0;
    r(3) = 0.0;
    // r(3) = 3;

    ocp.minimizeLSQ( S, h, r );

    std::cout<<"Solver defined"<<std::endl;


    OptimizationAlgorithm solver_(ocp);


    ////////////////// INITIALIZATION //////////////////////////////////

    std::cout<<"Initializing solver"<<std::endl;

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


    solver_.initializeDifferentialStates( state_init );
    solver_.initializeControls          ( control_init );
    // solver_.initializeAlgebraicStates(inter_state_init);
    // csv<<state_init<<std::endl;
    // csv<<control_init<<std::endl;


    //solver_.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver_.set( INTEGRATOR_TOLERANCE , 1e-8            );
    //solver_.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver_.set( KKT_TOLERANCE        , 1e-3            );

    // solver_.set( MAX_NUM_ITERATIONS        , 5  );

    // call the solver
    returnValue value = solver_.solve();
    // get solution
    VariablesGrid output_states,output_control;

    solver_.getDifferentialStates(output_states);
    solver_.getControls          (output_control);
    // solver_.getAlgebraicStates(algebraic_states);
    //output_states.print();
    int success_value = value;

    if(success_value == 0){
        for(int i=0;i<time_horizon_;i++){

        x_ptr_[i]=output_states(i,0);
        y_ptr_[i]=output_states(i,1);
        z_ptr_[i]=output_states(i,2);
        vx_ptr_[i]=output_states(i,3);
        vy_ptr_[i]=output_states(i,4);
        vz_ptr_[i]=output_states(i,5);
        ax_ptr_[i]=output_control(i,0);
        ay_ptr_[i]=output_control(i,1);
        az_ptr_[i]=output_control(i,2);
        // csv<<output_control(i,3)<<std::endl;
        }
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
    s.clearStaticCounters();
    // pitch.clearStaticCounters();

    return success_value;  
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
