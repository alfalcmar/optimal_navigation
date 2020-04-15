#include <ACADO.h>
//#include <ACADO/acado_optimal_control.hpp>

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



class acadoSolver{
    
    acadoSolver(){
        // define the model
        model << dot(px) == vx;
        model << dot(py) == vy;
        model << dot(pz) == vz;
        model << dot(vx) == ax;
        model << dot(vy) == ay;
        model << dot(vz) == az;
        // define the ocp
            // DEFINE AN OPTIMAL CONTROL PROBLEM:
        // ----------------------------------
        OCP ocp( t_start, t_end, 20 );
        ocp.minimizeLagrangeTerm();  // weight this with the physical cost!!!
        ocp.subjectTo( f );

        ocp.subjectTo( AT_START, px ==  6.5 );
        ocp.subjectTo( AT_START, py == 12.0 );
        ocp.subjectTo( AT_START, pz == 22.0 );
        ocp.subjectTo( AT_START, vx == 22.0 );
        ocp.subjectTo( AT_START, vy == 22.0 );
        ocp.subjectTo( AT_START, vz == 22.0 );

        ocp.subjectTo( Sfmin <= Sf <= Sfmax );

        // define the solver
        OptimizationAlgorithm solver(ocp);
        solver.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN ); // set the solver

        ////////////////// INITIALIZATION //////////////////////////////////
        
        Grid timeGrid( 0.0, 1.0, 11 );
    
        VariablesGrid   px_init(timeGrid );
        VariablesGrid   py_init(timeGrid );
        VariablesGrid   pz_init(timeGrid );
    
        x_init(0,0 ) = 0.00e+00; x_init(1,0 ) = 0.00e+00; x_init(2,0 ) = 1.00e+00;   
        x_init(0,1 ) = 2.99e-01; x_init(1,1 ) = 7.90e-01; x_init(2,1 ) = 9.90e-01; 
        x_init(0,2 ) = 1.13e+00; x_init(1,2 ) = 1.42e+00; x_init(2,2 ) = 9.81e-01; 
        x_init(0,3 ) = 2.33e+00; x_init(1,3 ) = 1.69e+00; x_init(2,3 ) = 9.75e-01; 
        x_init(0,4 ) = 3.60e+00; x_init(1,4 ) = 1.70e+00; x_init(2,4 ) = 9.73e-01; 
        x_init(0,5 ) = 4.86e+00; x_init(1,5 ) = 1.70e+00; x_init(2,5 ) = 9.70e-01; 
        x_init(0,6 ) = 6.13e+00; x_init(1,6 ) = 1.70e+00; x_init(2,6 ) = 9.68e-01; 
        x_init(0,7 ) = 7.39e+00; x_init(1,7 ) = 1.70e+00; x_init(2,7 ) = 9.65e-01; 
        x_init(0,8 ) = 8.66e+00; x_init(1,8 ) = 1.70e+00; x_init(2,8 ) = 9.63e-01; 
        x_init(0,9 ) = 9.67e+00; x_init(1,9 ) = 8.98e-01; x_init(2,9 ) = 9.58e-01; 
        x_init(0,10) = 1.00e+01; x_init(1,10) = 0.00e+00; x_init(2,10) = 9.49e-01;
        
        u_init(0,0 ) =  1.10e+00;   
        u_init(0,1 ) =  1.10e+00;
        u_init(0,2 ) =  1.10e+00;
        u_init(0,3 ) =  5.78e-01;
        u_init(0,4 ) =  5.78e-01;
        u_init(0,5 ) =  5.78e-01;
        u_init(0,6 ) =  5.78e-01;
        u_init(0,7 ) =  5.78e-01;
        u_init(0,8 ) = -2.12e-01;
        u_init(0,9 ) = -1.10e+00;
        u_init(0,10) = -1.10e+00;
        
        p_init(0,0 ) =  7.44e+00;
        
        algorithm.initializeDifferentialStates( x_init );
        algorithm.initializeControls          ( u_init );
        algorithm.initializeParameters        ( p_init );

/////////////////////////////////////////////////////////////////////////////////////////////

        algorithm.set( MAX_NUM_ITERATIONS, 20 );
        algorithm.set( KKT_TOLERANCE, 1e-8 );

        VariablesGrid output_states,output_control;
        algorithm.getDifferentialStates(output_states);
        algorithm.getControls          (output_control);
    

    }
    private:

    DifferentialState px,py,pz,vx,vy,vz;
    Control ax,ay,az;
    DifferentialEquation model;   // Allows to setup and evaluate differential equations


};

 
int solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,const nav_msgs::Odometry &desired_odometry, const std::array<float,2> &obst, const std::vector<nav_msgs::Odometry> &target_trajectory, std::map<int,nav_msgs::Odometry> &uavs_pose, int drone_id, bool target,bool multi){

ROS_INFO("solverFunction");

}







