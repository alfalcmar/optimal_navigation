#include <ACADO.h>
#include <chrono>

// /** \brief Utility function to save parameters of the solver in a CSV file
//  *  \param params       these params were sent to the solver
//  */
// void saveParametersToCsv(const FORCESNLPsolver_params &params){
//     const int x = 0;
//     const int y = 1;
//     csv_debug<<"Number of params: "<<sizeof(params.all_parameters)/sizeof(params.all_parameters[0])<<std::endl;
//     // logging initial constarint
//     csv_debug<<"initial constraint: "<<params.xinit[0]<<", "<<params.xinit[1]<<", "<<params.xinit[2]<<", "<<params.xinit[3]<<", "<<params.xinit[4]<<", "<<params.xinit[5]<<std::endl;

//     //logging initial guess
//     csv_debug<<std::endl<<std::endl<<"Initial guess: "<<std::endl;
//     csv_debug<<"Number os guesses: "<<sizeof(params.x0)/sizeof(params.x0[0]);
//     for(int i=0; i<sizeof(params.x0)/sizeof(params.x0[0]);i++){
//         csv_debug<<params.x0[i]<<std::endl;
//     }

//     for(int j=0; j<time_horizon;j++){
//         for(int i=npar*j; i<npar*j+npar;i++){
//             csv_debug << params.all_parameters[i] << ", ";
//         }
//         csv_debug<<std::endl;
//     }
// }

USING_NAMESPACE_ACADO



    
ACADOsolver::ACADOsolver(){
    
    ROS_INFO("calling solver function");
    DifferentialEquation model;
    ROS_INFO("Acado constructor");
    // // define the model
    // model << dot(px_) == vx_;
    // model << dot(py_) == vy_;
    // model << dot(pz_) == vz_;
    // model << dot(vx_) == ax_;
    // model << dot(vy_) == ay_;
    // model << dot(vz_) == az_;
    // define the ocp
        // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    
    //ocp.minimizeLagrangeTerm(ux*ux+uy*uy);  // weight this with the physical cost!!!
    // TODO parameters
    // ocp.minimizeMayerTerm((desired_odometry.pose.pose.position.x-px)*(desired_odometry.pose.pose.position.x-px)+(desired_odometry.pose.pose.position.y-py)*(desired_odometry.pose.pose.position.y-py));
    // ocp.subjectTo(model);

    // // TODO parameters
    // ocp.subjectTo( AT_START, px ==  uavs_pose[drone_id].pose.pose.position.x);
    // ocp.subjectTo( AT_START, py == uavs_pose[drone_id].pose.pose.position.y);
    // ocp.subjectTo( AT_START, vx == uavs_pose[drone_id].twist.twist.linear.x);
    // ocp.subjectTo( AT_START, vy == uavs_pose[drone_id].twist.twist.linear.y);

    // //ocp.subjectTo( Sfmin <= Sf <= Sfmax );

    // // define the solver
    // OptimizationAlgorithm solver(ocp);
    // solver_ = new OptimizationAlgorithm(ocp);
    // solver_->set( HESSIAN_APPROXIMATION, EXACT_HESSIAN ); // set the solver
    // auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> diff = end-start;
    // std::cout << "Time to prepare the solver " <<diff.count() << " s\n";
    
    ////////////////// INITIALIZATION //////////////////////////////////
    
    // Grid timeGrid( 0.0, 1.0, 11 );

    // VariablesGrid   px_init(timeGrid );
    // VariablesGrid   py_init(timeGrid );
    // VariablesGrid   pz_init(timeGrid );

    // x_init(0,0 ) = 0.00e+00; x_init(1,0 ) = 0.00e+00; x_init(2,0 ) = 1.00e+00;   
    // x_init(0,1 ) = 2.99e-01; x_init(1,1 ) = 7.90e-01; x_init(2,1 ) = 9.90e-01; 
    // x_init(0,2 ) = 1.13e+00; x_init(1,2 ) = 1.42e+00; x_init(2,2 ) = 9.81e-01; 
    // x_init(0,3 ) = 2.33e+00; x_init(1,3 ) = 1.69e+00; x_init(2,3 ) = 9.75e-01; 
    // x_init(0,4 ) = 3.60e+00; x_init(1,4 ) = 1.70e+00; x_init(2,4 ) = 9.73e-01; 
    // x_init(0,5 ) = 4.86e+00; x_init(1,5 ) = 1.70e+00; x_init(2,5 ) = 9.70e-01; 
    // x_init(0,6 ) = 6.13e+00; x_init(1,6 ) = 1.70e+00; x_init(2,6 ) = 9.68e-01; 
    // x_init(0,7 ) = 7.39e+00; x_init(1,7 ) = 1.70e+00; x_init(2,7 ) = 9.65e-01; 
    // x_init(0,8 ) = 8.66e+00; x_init(1,8 ) = 1.70e+00; x_init(2,8 ) = 9.63e-01; 
    // x_init(0,9 ) = 9.67e+00; x_init(1,9 ) = 8.98e-01; x_init(2,9 ) = 9.58e-01; 
    // x_init(0,10) = 1.00e+01; x_init(1,10) = 0.00e+00; x_init(2,10) = 9.49e-01;
    
    // u_init(0,0 ) =  1.10e+00;   
    // u_init(0,1 ) =  1.10e+00;
    // u_init(0,2 ) =  1.10e+00;
    // u_init(0,3 ) =  5.78e-01;
    // u_init(0,4 ) =  5.78e-01;
    // u_init(0,5 ) =  5.78e-01;
    // u_init(0,6 ) =  5.78e-01;
    // u_init(0,7 ) =  5.78e-01;
    // u_init(0,8 ) = -2.12e-01;
    // u_init(0,9 ) = -1.10e+00;
    // u_init(0,10) = -1.10e+00;
    
    // p_init(0,0 ) =  7.44e+00;
    
   // solver.initializeDifferentialStates( x_init );
   // solver.initializeControls          ( u_init );
    //solver.initializeParameters        ( p_init );

/////////////////////////////////////////////////////////////////////////////////////////////




}
// private:

// DifferentialState px,py,pz,vx,vy,vz;
// Control ax,ay,az;
// DifferentialEquation model;   // Allows to setup and evaluate differential equations
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

// }


 int ACADOsolver::solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,nav_msgs::Odometry &desired_odometry, const std::array<float,2> &obst, const std::vector<nav_msgs::Odometry> &target_trajectory, std::map<int,nav_msgs::Odometry> &uavs_pose, int drone_id, bool target,bool multi){
    DifferentialState px_,py_,pz_,vx_,vy_,vz_;
    Control ax_,ay_,az_;
    Parameter tx,ty;
    auto start = std::chrono::system_clock::now();
    ROS_INFO("calling solver function");
    DifferentialEquation model;
    //AlgebraicState pitch;
    ROS_INFO("Acado constructor");

    //pitch = 2*atan2(pz_/(sqrt(pow((px_-tx),2) + pow((py_-ty),2)+pow(pz_,2))+sqrt(pow((px_-tx),2) + pow((py_-ty),2))));
    // define the model
    model << dot(px_) == vx_;
    model << dot(py_) == vy_;
    model << dot(pz_) == vz_;
    model << dot(vx_) == ax_;
    model << dot(vy_) == ay_;
    model << dot(vz_) == az_;
    Grid myGrid( t_start,t_end,N );
    myGrid.print();
    VariablesGrid target_x(1,myGrid);
    VariablesGrid target_y(1,myGrid);
    OCP ocp_( myGrid); // possibility to set non equidistant time-horizon of the problem
    ocp_.subjectTo(model);
    ocp_.subjectTo( -5.0 <= ax_ <=  5.0   );  
    ocp_.subjectTo(  -5.0 <= ay_ <= 5.0   );
    ocp_.subjectTo(  -5.0 <= az_ <= 5.0   );
    ocp_.subjectTo(  -50.0 <= px_ <= 50.0   );
    ocp_.subjectTo(  -50.0 <= py_ <= 50.0   );
    ocp_.subjectTo(  0.0 <= pz_ <= 20.0   );
    ocp_.subjectTo(  -1.0 <= vx_ <= 1.0   );
    ocp_.subjectTo(  -1.0 <= vy_ <= 1.0   );
    ocp_.subjectTo(  -1.0 <= vz_ <= 1.0   );
    ocp_.subjectTo( tx==target_x);
    ocp_.subjectTo( ty==target_y);
    //ocp_.subjectTo( -M_PI_4 <=pitch <= M_PI_2); //pitch constraint
    checkConstraints(desired_odometry,uavs_pose);

   // ocp_.minimizeLagrangeTerm(ax*ax+ay*ay);  // weight this with the physical cost!!!
    
    ocp_.subjectTo( AT_START, px_ == uavs_pose.at(drone_id).pose.pose.position.x);
    ocp_.subjectTo( AT_START, py_ == uavs_pose.at(drone_id).pose.pose.position.y);
    ocp_.subjectTo( AT_START, pz_ == uavs_pose.at(drone_id).pose.pose.position.z);
    ocp_.subjectTo( AT_START, vx_== uavs_pose.at(drone_id).twist.twist.linear.x);
    ocp_.subjectTo( AT_START, vy_== uavs_pose.at(drone_id).twist.twist.linear.y);
    ocp_.subjectTo( AT_START, vz_== uavs_pose.at(drone_id).twist.twist.linear.z);


    ocp_.minimizeMayerTerm((desired_odometry.pose.pose.position.x-px_)*(desired_odometry.pose.pose.position.x-px_)+
                            (desired_odometry.pose.pose.position.y-py_)*(desired_odometry.pose.pose.position.y-py_)+
                            (desired_odometry.pose.pose.position.z-pz_)*(desired_odometry.pose.pose.position.z-pz_));
    ocp_.minimizeLagrangeTerm(ax_*ax_+ay_*ay_+az_*az_);


    ROS_INFO("objective function defined");
    // TODO parameters

    // ocp_.subjectTo( 0, vy == uavs_pose.at(drone_id).twist.twist.linear.y);
    //ocp_.subjectTo( 'AT_START', vx == 0.0);
    //ocp_.subjectTo( AT_START, vy == 0.0);
 

    //ocp.subjectTo( Sfmin <= Sf <= Sfmax );
    ROS_INFO("defining solver");
    // define the solver
    LogRecord logRecord(LOG_AT_EACH_ITERATION);
    logRecord << LOG_NUM_NLP_ITERATIONS;
    logRecord << LOG_KKT_TOLERANCE;
    //logRecord << LOG_OBJECTIVE_FUNCTION;
    logRecord << LOG_MERIT_FUNCTION_VALUE;
    logRecord << LOG_LINESEARCH_STEPLENGTH;
    //logRecord << LOG_ALGREBRAIC_STATES;
    logRecord << LOG_CONTROLS;
    logRecord << LOG_DISTURBANCES;
    logRecord << LOG_INTERMEDIATE_STATES;
    logRecord << LOG_DIFFERENTIAL_STATES;


    OptimizationAlgorithm solver_(ocp_);

    solver_ << logRecord;
    //solver_.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN ); // set the solver
    //solver_.set( MAX_NUM_ITERATIONS, 400 );
    solver_.set( KKT_TOLERANCE, 1e-5 );
    //solver_.set( INTEGRATOR_TYPE      , INT_RK78        );
    solver_.set( INTEGRATOR_TOLERANCE , 1e-8            );
    //solver_.set( DISCRETIZATION_TYPE  , SINGLE_SHOOTING );
    solver_.set( KKT_TOLERANCE        , 1e-4            );
    

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;
    std::cout << "Time to prepare the solver " <<diff.count() << " s\n";

    start = std::chrono::system_clock::now();
    returnValue value = solver_.solve();
    std::cout<<value<<std::endl;
    end = std::chrono::system_clock::now();
    diff = end-start;
    std::cout << "Time to solve " <<diff.count() << " s\n";
    VariablesGrid output_states,output_control;
    // get states

    solver_.getDifferentialStates(output_states);
    solver_.getControls          (output_control);
    output_states.print();
    for(int i=0;i<N;i++){
        x.push_back(output_states(i,0));
        y.push_back(output_states(i,1));
        z.push_back(output_states(i,2));
        vx.push_back(output_states(i,3));
        vy.push_back(output_states(i,4));
        vz.push_back(output_states(i,5));
    }

    ROS_INFO("output saved");
    px_.clearStaticCounters();
    py_.clearStaticCounters();
    pz_.clearStaticCounters();
    vx_.clearStaticCounters();
    vy_.clearStaticCounters();
    vz_.clearStaticCounters();
    ax_.clearStaticCounters();
    ay_.clearStaticCounters();
    az_.clearStaticCounters();

//     std::cout<<"printin prueba: "<<prueba<<std::endl;
//     output_states.print();
//    // GnuplotWindow window;
//    // window.addSubplot('output_control');
//    // window.plot();
    return 1;    
 }

// ROS_INFO("solverFunction");

// }







