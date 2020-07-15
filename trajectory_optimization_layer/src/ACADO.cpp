#include <ACADO.h>
#include <chrono>


USING_NAMESPACE_ACADO

    
ACADOsolver::ACADOsolver(){
    
    ROS_INFO("Acado constructor");
    std::string mypackage = ros::package::getPath("optimal_control_interface");
    csv.open(mypackage+"/logs/"+"acado_log_"+std::to_string(drone_id_)+".csv");
    //my_grid_ = new Grid( t_start,t_end,N );
    //my_grid_->print();
}

void ACADOsolver::checkConstraints(nav_msgs::Odometry &desired_odometry, std::map<int,nav_msgs::Odometry> &uavs_pose){
    //pose
    //velocity
    if(uavs_pose[drone_id_].twist.twist.linear.x>1.0){
        ROS_WARN("x velocity out of bound");
        uavs_pose[drone_id_].twist.twist.linear.x=1;
    }else if(uavs_pose[drone_id_].twist.twist.linear.x<-1.0){
        uavs_pose[drone_id_].twist.twist.linear.x=-1;
        ROS_WARN("x velocity out of bound");
    }
    if(uavs_pose[drone_id_].twist.twist.linear.y>1.0){
        uavs_pose[drone_id_].twist.twist.linear.y=1;
        ROS_WARN("y velocity out of bound");
    }else if(uavs_pose[drone_id_].twist.twist.linear.y<-1.0){
        uavs_pose[drone_id_].twist.twist.linear.y=-1;
        ROS_WARN("y velocity out of bound");
    }
    if(uavs_pose[drone_id_].twist.twist.linear.z>1.0){
        uavs_pose[drone_id_].twist.twist.linear.z=-1;
        ROS_WARN("z velocity out of bound");
    }else if(uavs_pose[drone_id_].twist.twist.linear.z<-1.0){
        uavs_pose[drone_id_].twist.twist.linear.z=-1;
        ROS_WARN("z velocity out of bound");
    }
}

int ACADOsolver::checkTime(){
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    csv<<"time solving: "<<diff.count()<<std::endl;
    if(diff.count()>3){
        return 0;
    }else{
        int number_steps = (diff.count()/step_size);
        return number_steps;
    }
    csv<<"time solving: "<<diff.count()<<std::endl;

}

int ACADOsolver::solverFunction2D(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,std::array<double,TIME_HORIZON> &_ax, std::array<double,TIME_HORIZON> &_ay, std::array<double,TIME_HORIZON> &_az,std::array<double,TIME_HORIZON> &_x, std::array<double,TIME_HORIZON> &_y, std::array<double,TIME_HORIZON> &_z,std::array<double,TIME_HORIZON> &_vx, std::array<double,TIME_HORIZON> &_vy, std::array<double,TIME_HORIZON> &_vz,nav_msgs::Odometry &_desired_odometry, const std::array<float,2> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, int number_steps, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){
    
    int number_steps_1 = checkTime();
    DifferentialState px_,py_,vx_,vy_;
    //DifferentialState   dummy;  // dummy state
    Control ax_,ay_;

    //Control s  ;  // slack variable

    // Parameter tx,ty;
    ROS_INFO("calling solver function");
    DifferentialEquation model;
    // AlgebraicState pitch;
    ROS_INFO("Acado constructor");
    Grid my_grid_( t_start,t_end,N );

  
    
    //pitch = 2*atan(_uavs_pose[drone_id_].pose.pose.position.z/(sqrt(pow((px_-tx),2) + pow((py_-ty),2)+pow(_uavs_pose[drone_id_].pose.pose.position.z,2))+sqrt(pow((px_-tx),2) + pow((py_-ty),2))));
    // define the model
    model << dot(px_) == vx_;
    model << dot(py_) == vy_;
    model << dot(vx_) == ax_;
    model << dot(vy_) == ay_;
    
    OCP ocp(my_grid_);// = new OCP( my_grid_); // possibility to set non equidistant time-horizon of the problem
    ocp.subjectTo(model);
    ocp.subjectTo( -5.0 <= ax_ <=  5.0   );  
    ocp.subjectTo(  -5.0 <= ay_ <= 5.0   );
    // ocp.subjectTo(  -50.0 <= px_ <= 50.0   );
    // ocp.subjectTo(  -50.0 <= py_ <= 50.0   );
    ocp.subjectTo(  -1.0 <= vx_ <= 1.0   );
    ocp.subjectTo(  -1.0 <= vy_ <= 1.0   );
    
    // target set-up
    VariablesGrid target_x(1,my_grid_);
    VariablesGrid target_y(1,my_grid_);

    // //set target trajectory
    // for(uint i=0; i<N; i++){
    //     target_x(i,0)=_target_trajectory[i].pose.pose.position.x;
    //     target_y(i,0)=_target_trajectory[i].pose.pose.position.y;
    // }

    // ocp.subjectTo( tx==target_x);
    // ocp.subjectTo( ty==target_y);

    
    
    /** pitch constraint formulation
    * y = sqrt(pow((px_-tx),2) + pow((py_-ty),2))
    * x= _uavs_pose[drone_id_].pose.pose.position.z
    * pitch = atan2(y,x)
    * atan2 = 2*atan(y/(sqrt(x^2+y^2)+x))
    * ocp.subjectTo( -M_PI_4 <=pitch <= M_PI_2); //pitch constraint
    */

    // ocp.subjectTo(-M_PI_4<=2*atan(sqrt(pow((px_-tx),2) + pow((py_-ty),2))/(sqrt(pow(_uavs_pose[drone_id_].pose.pose.position.z,2)+
            // pow((px_-tx),2) + pow((py_-ty),2))+_uavs_pose[drone_id_].pose.pose.position.z))<=M_PI_2);

    checkConstraints(_desired_odometry,_uavs_pose);    

    csv<<"first time: "<<first_time_solving<<std::endl; // check it
    
    if(first_time_solving){
       ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).pose.pose.position.x);
       ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).pose.pose.position.y);
    }else{
        ocp.subjectTo( AT_START, px_ == _x[number_steps+1]);
        ocp.subjectTo( AT_START, py_ == _y[number_steps+1]);
        ocp.subjectTo( 1, px_ == _x[number_steps+2]);
        ocp.subjectTo( 1, py_ == _y[number_steps+2]);
        ocp.subjectTo( 2, px_ == _x[number_steps+3]);
        ocp.subjectTo( 2, py_ == _y[number_steps+3]);
        ocp.subjectTo( 3, px_ == _x[number_steps+4]);
        ocp.subjectTo( 3, py_ == _y[number_steps+4]);
    }
    // ocp.subjectTo( AT_START, vx_== 0.5*(_desired_odometry.pose.pose.position.x-_uavs_pose.at(_drone_id).pose.pose.position.x)/
                                    // sqrt(pow((_desired_odometry.pose.pose.position.x-_uavs_pose.at(_drone_id).pose.pose.position.x),2)+pow((_desired_odometry.pose.pose.position.y-_uavs_pose.at(_drone_id).pose.pose.position.y),2)));//_uavs_pose.at(_drone_id).twist.twist.linear.x);
    // ocp.subjectTo( AT_START, vy_== 0.5*(_desired_odometry.pose.pose.position.y-_uavs_pose.at(_drone_id).pose.pose.position.y)/
                                    // sqrt(pow((_desired_odometry.pose.pose.position.x-_uavs_pose.at(_drone_id).pose.pose.position.x),2)+pow((_desired_odometry.pose.pose.position.y-_uavs_pose.at(_drone_id).pose.pose.position.y),2)));//_uavs_pose.at(_drone_id).twist.twist.linear.y);
    // ocp.subjectTo( AT_START, ax_== _ax[number_steps+1]); 
    // ocp.subjectTo( AT_START, ay_== _ay[number_steps+1]);
    //ocp.subjectTo( s >= 0 ); slack variable

    // DEFINE LSQ function to minimize the end distance to the desired pose
    Function h_1;

    h_1 << px_;
    h_1 << py_;
    h_1 << vx_;
    h_1 << vy_;

    DMatrix S_1(4,4);
    DVector r_1(4);

    S_1.setIdentity();
	S_1(0,0) = 1.0;
	S_1(1,1) = 1.0;
	S_1(2,2) = 1.0;
	S_1(3,3) = 1.0;

    r_1(0) = _desired_odometry.pose.pose.position.x;
    r_1(1) = _desired_odometry.pose.pose.position.y;
    r_1(2) = _desired_odometry.twist.twist.linear.x;
    r_1(3) = _desired_odometry.twist.twist.linear.y;

    ocp.minimizeLSQEndTerm( S_1, h_1, r_1 );


    // ocp.minimizeMayerTerm((_desired_odometry.pose.pose.position.x-px_)*(_desired_odometry.pose.pose.position.x-px_)+
    //                         (_desired_odometry.pose.pose.position.y-py_)*(_desired_odometry.pose.pose.position.y-py_));
    
    // DEFINE LSQ function to minimize accelerations
    Function h;

    h << ax_;
    h << ay_;

    DMatrix S(2,2);
    DVector r(2);

    S.setIdentity();
	S(0,0) = 1.0;
	S(1,1) = 1.0;

    r.setAll( 0.0 );

    ocp.minimizeLSQ( S, h, r );


    //ocp.minimizeLagrangeTerm(ax_*ax_+ay_*ay_/* + cinematography term */ /*+s*/);


    ROS_INFO("objective function defined");
    // TODO parameters

    float radius = 4.0;
    ocp.subjectTo(radius*radius <=  px_*px_+py_*py_ /*+s*/);
    ROS_INFO("defining solver");
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

    OptimizationAlgorithm solver_(ocp);


    ////////////////// INITIALIZATION //////////////////////////////////
    ROS_INFO("initializing");

    VariablesGrid state_init(6,my_grid_), control_init(3,my_grid_);
   
    for(uint i=0; i<N; i++){
        control_init(i,0)=_initial_guess["ax"][i];
        control_init(i,1)=_initial_guess["ay"][i];
        state_init(i,0)=_initial_guess["px"][i];
        state_init(i,1)=_initial_guess["py"][i];
        state_init(i,3)=_initial_guess["vx"][i];
        state_init(i,4)=_initial_guess["vy"][i];
    }

    solver_.initializeDifferentialStates( state_init );
    solver_.initializeControls          ( control_init );
    // csv<<state_init<<std::endl;
    // csv<<control_init<<std::endl;


    //solver_.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver_.set( INTEGRATOR_TOLERANCE , 1e-8            );
    //solver_.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver_.set( KKT_TOLERANCE        , 1e-3            );
    

    start = std::chrono::system_clock::now();
    
    // call the solver
    returnValue value = solver_.solve();
    // get solution
    VariablesGrid output_states,output_control;

    solver_.getDifferentialStates(output_states);
    solver_.getControls          (output_control);
    //output_states.print();
    for(int i=0;i<N;i++){
         _x[i]=output_states(i,0);
         _y[i]=output_states(i,1);
         _z[i]= _desired_odometry.pose.pose.position.z;
        _vx[i]=output_states(i,3);
        _vy[i]=output_states(i,4);
        _vz[i]=0.0;
        _ax[i]=output_control(i,0);
        _ay[i]=output_control(i,1);
        _az[i]= output_control(i,2);
    }

    px_.clearStaticCounters();
    py_.clearStaticCounters();
    vx_.clearStaticCounters();
    vy_.clearStaticCounters();
    ax_.clearStaticCounters();
    ay_.clearStaticCounters();
    //s.clearStaticCounters(); slack variable
    int success_value = value;
    return success_value;
}
int ACADOsolver::solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,std::array<double,TIME_HORIZON> &_ax, std::array<double,TIME_HORIZON> &_ay, std::array<double,TIME_HORIZON> &_az,std::array<double,TIME_HORIZON> &_x, std::array<double,TIME_HORIZON> &_y, std::array<double,TIME_HORIZON> &_z,std::array<double,TIME_HORIZON> &_vx, std::array<double,TIME_HORIZON> &_vy, std::array<double,TIME_HORIZON> &_vz,nav_msgs::Odometry &_desired_odometry, const std::array<float,2> &_obst, const std::vector<nav_msgs::Odometry> &_target_trajectory, std::map<int,nav_msgs::Odometry> &_uavs_pose, int number_steps, bool first_time_solving, int _drone_id, bool _target /*false*/,bool _multi/*false*/){
    DifferentialState px_,py_,pz_,vx_,vy_,vz_;
    //DifferentialState   dummy;  // dummy state
    Control ax_,ay_,az_;

    //Control s  ;  // slack variable
    int number_steps_1 = checkTime();

    
    //Parameter tx,ty;
    auto start = std::chrono::system_clock::now();
    ROS_INFO("calling solver function");
    DifferentialEquation model;
    //AlgebraicState pitch;
    ROS_INFO("Acado constructor");
    Grid my_grid_( t_start,t_end,N );

    //pitch = 2*atan2(pz_/(sqrt(pow((px_-tx),2) + pow((py_-ty),2)+pow(pz_,2))+sqrt(pow((px_-tx),2) + pow((py_-ty),2))));
    // define the model
    model << dot(px_) == vx_;
    model << dot(py_) == vy_;
    model << dot(pz_) == vz_;
    model << dot(vx_) == ax_;
    model << dot(vy_) == ay_;
    model << dot(vz_) == az_;
    //VariablesGrid target_x(1,myGrid);
    //VariablesGrid target_y(1,myGrid);
    OCP ocp(my_grid_);// = new OCP( my_grid_); // possibility to set non equidistant time-horizon of the problem
    ocp.subjectTo(model);
    ocp.subjectTo( -5.0 <= ax_ <=  5.0   );  
    ocp.subjectTo(  -5.0 <= ay_ <= 5.0   );
    ocp.subjectTo(  -5.0 <= az_ <= 5.0   );
    // ocp.subjectTo(  -50.0 <= px_ <= 50.0   );
    // ocp.subjectTo(  -50.0 <= py_ <= 50.0   );
    ocp.subjectTo(  1.0 <= pz_ <= 20.0   );
    ocp.subjectTo(  -1.0 <= vx_ <= 1.0   );
    ocp.subjectTo(  -1.0 <= vy_ <= 1.0   );
    ocp.subjectTo(  -1.0 <= vz_ <= 1.0   );
    //ocp.subjectTo( tx==target_x);
    //ocp.subjectTo( ty==target_y);
    //ocp.subjectTo( -M_PI_4 <=pitch <= M_PI_2); //pitch constraint
    checkConstraints(_desired_odometry,_uavs_pose);
    csv<<"first time: "<<first_time_solving<<std::endl; // check it

    // ocp.minimizeLagrangeTerm(ax*ax+ay*ay);  // weight this with the physical cost!!!
    if(first_time_solving){
        ocp.subjectTo( AT_START, px_ == _uavs_pose.at(_drone_id).pose.pose.position.x);
        ocp.subjectTo( AT_START, py_ == _uavs_pose.at(_drone_id).pose.pose.position.y);
        ocp.subjectTo( AT_START, pz_ == _uavs_pose.at(_drone_id).pose.pose.position.z);
    }else{
        ocp.subjectTo( AT_START, px_ == _x[number_steps+1]);
        ocp.subjectTo( AT_START, py_ == _y[number_steps+1]);
        ocp.subjectTo( AT_START, pz_ == _z[number_steps+1]);
        ocp.subjectTo( 1, px_ == _x[number_steps+2]);
        ocp.subjectTo( 1, py_ == _y[number_steps+2]);
        ocp.subjectTo( 1, pz_ == _z[number_steps+2]);
        ocp.subjectTo( 2, px_ == _x[number_steps+3]);
        ocp.subjectTo( 2, py_ == _y[number_steps+3]);
        ocp.subjectTo( 2, pz_ == _z[number_steps+3]);
        ocp.subjectTo( 3, px_ == _x[number_steps+4]);
        ocp.subjectTo( 3, py_ == _y[number_steps+4]);
        ocp.subjectTo( 3, pz_ == _z[number_steps+4]);
    }
    // ocp.subjectTo( AT_START, vx_== _vx[number_steps]);//_uavs_pose.at(_drone_id).twist.twist.linear.x);
    // ocp.subjectTo( AT_START, vy_== _vy[number_steps]);//_uavs_pose.at(_drone_id).twist.twist.linear.y);
    // ocp.subjectTo( AT_START, vz_== _vz[number_steps]);//_uavs_pose.at(_drone_id).twist.twist.linear.z);

    //ocp.subjectTo( s >= 0 ); slack variable

    // DEFINE LSQ function to minimize the end distance to the desired pose
    Function h_1;

    h_1 << px_;
    h_1 << py_;
    h_1 << pz_;
    h_1 << vx_;
    h_1 << vy_;
    h_1 << vz_;

    DMatrix S_1(6,6);
    DVector r_1(6);

    S_1.setIdentity();
	S_1(0,0) = 1.0;
	S_1(1,1) = 1.0;
	S_1(2,2) = 1.0;
	S_1(3,3) = 1.0;
	S_1(4,4) = 1.0;
	S_1(5,5) = 1.0;

    r_1(0) = _desired_odometry.pose.pose.position.x;
    r_1(1) = _desired_odometry.pose.pose.position.y;
    r_1(2) = _desired_odometry.pose.pose.position.z;
    r_1(3) = _desired_odometry.twist.twist.linear.x;
    r_1(4) = _desired_odometry.twist.twist.linear.y;
    r_1(5) = _desired_odometry.twist.twist.linear.z;

    ocp.minimizeLSQEndTerm( S_1, h_1, r_1 );


    // ocp.minimizeMayerTerm((_desired_odometry.pose.pose.position.x-px_)*(_desired_odometry.pose.pose.position.x-px_)+
    //                         (_desired_odometry.pose.pose.position.y-py_)*(_desired_odometry.pose.pose.position.y-py_)+
    //                         (_desired_odometry.pose.pose.position.z-pz_)*(_desired_odometry.pose.pose.position.z-pz_));
    ocp.minimizeLagrangeTerm(ax_*ax_+ay_*ay_+az_*az_/*+s*/);


    ROS_INFO("objective function defined");
    // TODO parameters

    float radius = 4.0;
    //ocp.subjectTo(radius*radius <=  px_*px_+py_*py_ /*+s*/);
    ROS_INFO("defining solver");
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

    OptimizationAlgorithm solver_(ocp);


    ////////////////// INITIALIZATION //////////////////////////////////
    ROS_INFO("initializing");

    VariablesGrid state_init(6,my_grid_), control_init(3,my_grid_);
   
    for(uint i=0; i<N; i++){
        control_init(i,0)=_initial_guess["ax"][i];
        control_init(i,1)=_initial_guess["ay"][i];
        control_init(i,2)=_initial_guess["az"][i];
        state_init(i,0)=_initial_guess["px"][i];
        state_init(i,1)=_initial_guess["py"][i];
        state_init(i,2)=_initial_guess["pz"][i];
        state_init(i,3)=_initial_guess["vx"][i];
        state_init(i,4)=_initial_guess["vy"][i];
        state_init(i,5)=_initial_guess["vz"][i];
    }

    solver_.initializeDifferentialStates( state_init );
    solver_.initializeControls          ( control_init );
    // csv<<state_init<<std::endl;
    // csv<<control_init<<std::endl;


    //solver_.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver_.set( INTEGRATOR_TOLERANCE , 1e-8            );
    //solver_.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver_.set( KKT_TOLERANCE        , 1e-3            );
    

    start = std::chrono::system_clock::now();
    
    // call the solver
    returnValue value = solver_.solve();
    // get solution
    VariablesGrid output_states,output_control;

    solver_.getDifferentialStates(output_states);
    solver_.getControls          (output_control);
    //output_states.print();
    for(int i=0;i<N;i++){
         _x[i]=output_states(i,0);
         _y[i]=output_states(i,1);
         _z[i]=output_states(i,2);
        _vx[i]=output_states(i,3);
        _vy[i]=output_states(i,4);
        _vz[i]=output_states(i,5);
        _ax[i]=output_control(i,0);
        _ay[i]=output_control(i,1);
        _az[i]=output_control(i,2);
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
   int success_value = value;
    return success_value;  
 }








