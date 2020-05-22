/** FORCES PRO c++ library*/
#include <FORCES_PRO.h>


#ifdef __cplusplus
extern "C"
{
#endif
    extern void FORCESNLPsolver_casadi2forces(double *x, double *y, double *l, double *p,
                          double *f, double *nabla_f, double *c, double *nabla_c,
                          double *h, double *nabla_h, double *H, int stage, int iteration);
#ifdef __cplusplus
}
#endif

const double TARGET_DIFF = 4.0;


FORCESPROsolver::FORCESPROsolver(){
    ROS_INFO("FORCES PRO solver constructor");
}
/** \brief Utility function to save parameters of the solver in a CSV file
 *  \param params       these params were sent to the solver
 */
void FORCESPROsolver::saveParametersToCsv(const FORCESNLPsolver_params &params){
    const int x = 0;
    const int y = 1;
    csv_debug<<"Number of params: "<<sizeof(params.all_parameters)/sizeof(params.all_parameters[0])<<std::endl;
    // logging initial constarint
    csv_debug<<"initial constraint: "<<params.xinit[0]<<", "<<params.xinit[1]<<", "<<params.xinit[2]<<", "<<params.xinit[3]<<", "<<params.xinit[4]<<", "<<params.xinit[5]<<std::endl;

    //logging initial guess
    csv_debug<<std::endl<<std::endl<<"Initial guess: "<<std::endl;
    csv_debug<<"Number os guesses: "<<sizeof(params.x0)/sizeof(params.x0[0]);
    for(int i=0; i<sizeof(params.x0)/sizeof(params.x0[0]);i++){
        csv_debug<<params.x0[i]<<std::endl;
    }

    for(int j=0; j<time_horizon;j++){
        for(int i=npar*j; i<npar*j+npar;i++){
            csv_debug << params.all_parameters[i] << ", ";
        }
        csv_debug<<std::endl;
    }
}

int FORCESPROsolver::solverFunction(std::map<std::string, std::array<double,TIME_HORIZON>> &_initial_guess,std::array<double,TIME_HORIZON> &ax, std::array<double,TIME_HORIZON> &ay, std::array<double,TIME_HORIZON> &az, std::array<double,TIME_HORIZON> &x, std::array<double,TIME_HORIZON> &y, std::array<double,TIME_HORIZON> &z, std::array<double,TIME_HORIZON> &vx, std::array<double,TIME_HORIZON> &vy, std::array<double,TIME_HORIZON> &vz,const nav_msgs::Odometry &desired_odometry, const std::array<float,2> &obst, const std::vector<nav_msgs::Odometry> &target_trajectory, std::map<int,nav_msgs::Odometry> &uavs_pose, int drone_id, bool target,bool multi){

    const int p_x = 0;
    const int p_y = 1;  
    /* declare FORCES variables and structures */
    FORCESNLPsolver_info myinfo;
    FORCESNLPsolver_params myparams;
    FORCESNLPsolver_output myoutput;
   
    /* define external function evaluating functions and derivatives (only for the high-level interface) */

    FORCESNLPsolver_extfunc pt2Function = &FORCESNLPsolver_casadi2forces;

    int i, exitflag;

    // set initial postion and velocity
    ros::spinOnce();
    myparams.xinit[0] = ax[solving_rate_*time_horizon/(step_size_*time_horizon)];
    myparams.xinit[1] = ay[solving_rate_*time_horizon/(step_size_*time_horizon)];
    myparams.xinit[2] = az[solving_rate_*time_horizon/(step_size_*time_horizon)];

    myparams.xinit[3] = uavs_pose[drone_id].pose.pose.position.x;
    myparams.xinit[4] = uavs_pose[drone_id].pose.pose.position.y;
    myparams.xinit[5] = 3.0;
 
    myparams.xinit[6] = uavs_pose[drone_id].twist.twist.linear.x;
    myparams.xinit[7] = uavs_pose[drone_id].twist.twist.linear.y;
    myparams.xinit[8] = uavs_pose[drone_id].twist.twist.linear.z;

    // set initial guess
    std::vector<double> x0;
    double x0i[] = {u_x, u_y, u_z, p_x, p_y, p_z, v_x, v_y, v_z};
    for (int j = 0; j < time_horizon; j++)
    {
        x0i[0]=_initial_guess["ax"][j];
        x0i[1]=_initial_guess["ay"][j];
        x0i[2]=_initial_guess["az"][j];
        x0i[3]=_initial_guess["px"][j];
        x0i[4]=_initial_guess["py"][j];
        x0i[5]=_initial_guess["pz"][j];
        x0i[6]=_initial_guess["vx"][j];
        x0i[7]=_initial_guess["vy"][j];
        x0i[8]=_initial_guess["vz"][j];
        for (i = 0; i < n_states_variables; i++)
        {
            x0.push_back(x0i[i]);
        }
    }
    for (i = 0; i < x0.size(); i++)
    {
        myparams.x0[i] = x0[i];
    }

    std::vector<double> params;

    // parameters
    for(int i=0;i<time_horizon; i++){
        // desired position
        params.push_back(desired_odometry.pose.pose.position.x);
        params.push_back(desired_odometry.pose.pose.position.y);
        params.push_back(3.0);
        
        // desired velocity
        params.push_back(desired_odometry.twist.twist.linear.x);
        params.push_back(desired_odometry.twist.twist.linear.y);
        params.push_back(0.0);
    
        if(target){ // if target included
            // set parameters

            //this function should be executed by action executer
            //calculateDesiredPoint(shooting_action_type, target_vel, desired_wp, desired_vel);
            // target velocity
            params.push_back(target_trajectory[i].twist.twist.linear.x);
            params.push_back(target_trajectory[i].twist.twist.linear.y);
            // target position
            params.push_back(target_trajectory[i].pose.pose.position.x);
            params.push_back(target_trajectory[i].pose.pose.position.y);
            
        }
        if(multi){
            std::map<int,std::vector<geometry_msgs::Point>>::iterator it;
            //TODO check priority
            /*int n_priority = 0;
            for(int j=0; j<priority.size();j++){
                if(drone_id == priority[j]){
                    n_priority = j;
                    break;
                }
            }  
            for(int j=0; j<priority.size();j++){
                if(j<n_priority){
                    params.push_back(uavs_trajectory[priority[j]][i].x);
                    params.push_back(uavs_trajectory[priority[j]][i].y);
                }else if(j>n_priority){
                    params.push_back(uavs_pose[priority[j]].pose.position.x);
                    params.push_back(uavs_pose[priority[j]].pose.position.y);
                }
            }*/
        }
        if(no_fly_zone){
            params.push_back(obst[0]);
            params.push_back(obst[1]);
        }   
        //ROS_INFO("Drone %d: Desired position x: %f y: %f z: %f", drone_id, desired_wp[0],desired_wp[1],desired_wp[2]);
        //ROS_INFO("Drone %d: Desired position x: %f y: %f z: %f", drone_id, desired_vel[0],desired_vel[1],desired_vel[2]);
        //ROS_INFO("Drone %d: target init f: %f y: %f", drone_id, target_vel[0],target_vel[1]);    
            
    }


    for(int i=0; i<params.size();i++) 
    {
        myparams.all_parameters[i] = params[i];
    }

    // call the solver

    if(debug){
        saveParametersToCsv(myparams);
    }
    exitflag = FORCESNLPsolver_solve(&myparams, &myoutput, &myinfo, stdout, pt2Function);
    // save the output in a vector
     ax[0]=myoutput.x01[acceleration_x];
     ax[1]=myoutput.x02[acceleration_x];
     ax[2]=myoutput.x03[acceleration_x];
     ax[3]=myoutput.x04[acceleration_x];
     ax[4]=myoutput.x05[acceleration_x];
     ax[5]=myoutput.x06[acceleration_x];
     ax[6]=myoutput.x07[acceleration_x];
     ax[7]=myoutput.x08[acceleration_x];
     ax[8]=myoutput.x09[acceleration_x];
     ax[9]=myoutput.x10[acceleration_x];
    ax[10]=myoutput.x11[acceleration_x];
    ax[11]=myoutput.x12[acceleration_x];
    ax[12]=myoutput.x13[acceleration_x];
    ax[13]=myoutput.x14[acceleration_x];
    ax[14]=myoutput.x15[acceleration_x];
    ax[15]=myoutput.x16[acceleration_x];
    ax[16]=myoutput.x17[acceleration_x];
    ax[17]=myoutput.x18[acceleration_x];
    ax[18]=myoutput.x19[acceleration_x];
    ax[19]=myoutput.x20[acceleration_x];
    ax[20]=myoutput.x21[acceleration_x];
    ax[21]=myoutput.x22[acceleration_x];
    ax[22]=myoutput.x23[acceleration_x];
    ax[23]=myoutput.x24[acceleration_x];
    ax[24]=myoutput.x25[acceleration_x];
    ax[25]=myoutput.x26[acceleration_x];
    ax[26]=myoutput.x27[acceleration_x];
    ax[27]=myoutput.x28[acceleration_x];
    ax[28]=myoutput.x29[acceleration_x];
    ax[29]=myoutput.x30[acceleration_x];
    ax[30]=myoutput.x31[acceleration_x];
    ax[31]=myoutput.x32[acceleration_x];
    ax[32]=myoutput.x33[acceleration_x];
    ax[33]=myoutput.x34[acceleration_x];
    ax[34]=myoutput.x35[acceleration_x];
    ax[35]=myoutput.x36[acceleration_x];
    ax[36]=myoutput.x37[acceleration_x];
    ax[37]=myoutput.x38[acceleration_x];
    ax[38]=myoutput.x39[acceleration_x];
    ax[39]=myoutput.x40[acceleration_x];


    ay[0]=myoutput.x01[acceleration_y];
    ay[1]=myoutput.x02[acceleration_y];
    ay[2]=myoutput.x03[acceleration_y];
    ay[3]=myoutput.x04[acceleration_y];
    ay[4]=myoutput.x05[acceleration_y];
    ay[5]=myoutput.x06[acceleration_y];
    ay[6]=myoutput.x07[acceleration_y];
    ay[7]=myoutput.x08[acceleration_y];
    ay[8]=myoutput.x09[acceleration_y];
    ay[9]=myoutput.x10[acceleration_y];
    ay[10]=myoutput.x11[acceleration_y];
    ay[11]=myoutput.x12[acceleration_y];
    ay[12]=myoutput.x13[acceleration_y];
    ay[13]=myoutput.x14[acceleration_y];
    ay[14]=myoutput.x15[acceleration_y];
    ay[15]=myoutput.x16[acceleration_y];
    ay[16]=myoutput.x17[acceleration_y];
    ay[17]=myoutput.x18[acceleration_y];
    ay[18]=myoutput.x19[acceleration_y];
    ay[19]=myoutput.x20[acceleration_y];
    ay[20]=myoutput.x21[acceleration_y];
    ay[21]=myoutput.x22[acceleration_y];
    ay[22]=myoutput.x23[acceleration_y];
    ay[23]=myoutput.x24[acceleration_y];
    ay[24]=myoutput.x25[acceleration_y];
    ay[25]=myoutput.x26[acceleration_y];
    ay[26]=myoutput.x27[acceleration_y];
    ay[27]=myoutput.x28[acceleration_y];
    ay[28]=myoutput.x29[acceleration_y];
    ay[29]=myoutput.x30[acceleration_y];
    ay[30]=myoutput.x31[acceleration_y];
    ay[31]=myoutput.x32[acceleration_y];
    ay[32]=myoutput.x33[acceleration_y];
    ay[33]=myoutput.x34[acceleration_y];
    ay[34]=myoutput.x35[acceleration_y];
    ay[35]=myoutput.x36[acceleration_y];
    ay[36]=myoutput.x37[acceleration_y];
    ay[37]=myoutput.x38[acceleration_y];
    ay[38]=myoutput.x39[acceleration_y];
    ay[39]=myoutput.x40[acceleration_y];

    az[0]=myoutput.x01[acceleration_z];
    az[1]=myoutput.x02[acceleration_z];
    az[2]=myoutput.x03[acceleration_z];
    az[3]=myoutput.x04[acceleration_z];
    az[4]=myoutput.x05[acceleration_z];
    az[5]=myoutput.x06[acceleration_z];
    az[6]=myoutput.x07[acceleration_z];
    az[7]=myoutput.x08[acceleration_z];
    az[8]=myoutput.x09[acceleration_z];
    az[9]=myoutput.x10[acceleration_z];
    az[10]=myoutput.x11[acceleration_z];
    az[11]=myoutput.x12[acceleration_z];
    az[12]=myoutput.x13[acceleration_z];
    az[13]=myoutput.x14[acceleration_z];
    az[14]=myoutput.x15[acceleration_z];
    az[15]=myoutput.x16[acceleration_z];
    az[16]=myoutput.x17[acceleration_z];
    az[17]=myoutput.x18[acceleration_z];
    az[18]=myoutput.x19[acceleration_z];
    az[19]=myoutput.x20[acceleration_z];
    az[20]=myoutput.x21[acceleration_z];
    az[21]=myoutput.x22[acceleration_z];
    az[22]=myoutput.x23[acceleration_z];
    az[23]=myoutput.x24[acceleration_z];
    az[24]=myoutput.x25[acceleration_z];
    az[25]=myoutput.x26[acceleration_z];
    az[26]=myoutput.x27[acceleration_z];
    az[27]=myoutput.x28[acceleration_z];
    az[28]=myoutput.x29[acceleration_z];
    az[29]=myoutput.x30[acceleration_z];
    az[30]=myoutput.x31[acceleration_z];
    az[31]=myoutput.x32[acceleration_z];
    az[32]=myoutput.x33[acceleration_z];
    az[33]=myoutput.x34[acceleration_z];
    az[34]=myoutput.x35[acceleration_z];
    az[35]=myoutput.x36[acceleration_z];
    az[36]=myoutput.x37[acceleration_z];
    az[37]=myoutput.x38[acceleration_z];
    az[38]=myoutput.x39[acceleration_z];
    az[39]=myoutput.x40[acceleration_z];

    x[0]=myoutput.x01[position_x];
    x[1]=myoutput.x02[position_x];
    x[2]=myoutput.x03[position_x];
    x[3]=myoutput.x04[position_x];
    x[4]=myoutput.x05[position_x];
    x[5]=myoutput.x06[position_x];
    x[6]=myoutput.x07[position_x];
    x[7]=myoutput.x08[position_x];
    x[8]=myoutput.x09[position_x];
    x[9]=myoutput.x10[position_x];
    x[10]=myoutput.x11[position_x];
    x[11]=myoutput.x12[position_x];
    x[12]=myoutput.x13[position_x];
    x[13]=myoutput.x14[position_x];
    x[14]=myoutput.x15[position_x];
    x[15]=myoutput.x16[position_x];
    x[16]=myoutput.x17[position_x];
    x[17]=myoutput.x18[position_x];
    x[18]=myoutput.x19[position_x];
    x[19]=myoutput.x20[position_x];
    x[20]=myoutput.x21[position_x];
    x[21]=myoutput.x22[position_x];
    x[22]=myoutput.x23[position_x];
    x[23]=myoutput.x24[position_x];
    x[24]=myoutput.x25[position_x];
    x[25]=myoutput.x26[position_x];
    x[26]=myoutput.x27[position_x];
    x[27]=myoutput.x28[position_x];
    x[28]=myoutput.x29[position_x];
    x[29]=myoutput.x30[position_x];
    x[30]=myoutput.x31[position_x];
    x[31]=myoutput.x32[position_x];
    x[32]=myoutput.x33[position_x];
    x[33]=myoutput.x34[position_x];
    x[34]=myoutput.x35[position_x];
    x[35]=myoutput.x36[position_x];
    x[36]=myoutput.x37[position_x];
    x[37]=myoutput.x38[position_x];
    x[38]=myoutput.x39[position_x];
    x[39]=myoutput.x40[position_x];
    
    
    



    vx[0]=myoutput.x01[velocity_x];
    vx[1]=myoutput.x02[velocity_x];
    vx[2]=myoutput.x03[velocity_x];
    vx[3]=myoutput.x04[velocity_x];
    vx[4]=myoutput.x05[velocity_x];
    vx[5]=myoutput.x06[velocity_x];
    vx[6]=myoutput.x07[velocity_x];
    vx[7]=myoutput.x08[velocity_x];
    vx[8]=myoutput.x09[velocity_x];
    vx[9]=myoutput.x10[velocity_x];
    vx[10]=myoutput.x11[velocity_x];
    vx[11]=myoutput.x12[velocity_x];
    vx[12]=myoutput.x13[velocity_x];
    vx[13]=myoutput.x14[velocity_x];
    vx[14]=myoutput.x15[velocity_x];
    vx[15]=myoutput.x16[velocity_x];
    vx[16]=myoutput.x17[velocity_x];
    vx[17]=myoutput.x18[velocity_x];
    vx[18]=myoutput.x19[velocity_x];
    vx[19]=myoutput.x20[velocity_x];
    vx[20]=myoutput.x21[velocity_x];
    vx[21]=myoutput.x22[velocity_x];
    vx[22]=myoutput.x23[velocity_x];
    vx[23]=myoutput.x24[velocity_x];
    vx[24]=myoutput.x25[velocity_x];
    vx[25]=myoutput.x26[velocity_x];
    vx[26]=myoutput.x27[velocity_x];
    vx[27]=myoutput.x28[velocity_x];
    vx[28]=myoutput.x29[velocity_x];
    vx[29]=myoutput.x30[velocity_x];
    vx[30]=myoutput.x31[velocity_x];
    vx[31]=myoutput.x32[velocity_x];
    vx[32]=myoutput.x33[velocity_x];
    vx[33]=myoutput.x34[velocity_x];
    vx[34]=myoutput.x35[velocity_x];
    vx[35]=myoutput.x36[velocity_x];
    vx[36]=myoutput.x37[velocity_x];
    vx[37]=myoutput.x38[velocity_x];
    vx[38]=myoutput.x39[velocity_x];
    vx[39]=myoutput.x40[velocity_x];
    
    

    y[0]=myoutput.x01[position_y];
    y[1]=myoutput.x02[position_y];
    y[2]=myoutput.x03[position_y];
    y[3]=myoutput.x04[position_y];
    y[4]=myoutput.x05[position_y];
    y[5]=myoutput.x06[position_y];
    y[6]=myoutput.x07[position_y];
    y[7]=myoutput.x08[position_y];
    y[8]=myoutput.x09[position_y];
    y[9]=myoutput.x10[position_y];
    y[10]=myoutput.x11[position_y];
    y[11]=myoutput.x12[position_y];
    y[12]=myoutput.x13[position_y];
    y[13]=myoutput.x14[position_y];
    y[14]=myoutput.x15[position_y];
    y[15]=myoutput.x16[position_y];
    y[16]=myoutput.x17[position_y];
    y[17]=myoutput.x18[position_y];
    y[18]=myoutput.x19[position_y];
    y[19]=myoutput.x20[position_y];
    y[20]=myoutput.x21[position_y];
    y[21]=myoutput.x22[position_y];
    y[22]=myoutput.x23[position_y];
    y[23]=myoutput.x24[position_y];
    y[24]=myoutput.x25[position_y];
    y[25]=myoutput.x26[position_y];
    y[26]=myoutput.x27[position_y];
    y[27]=myoutput.x28[position_y];
    y[28]=myoutput.x29[position_y];
    y[29]=myoutput.x30[position_y];
    y[30]=myoutput.x31[position_y];
    y[31]=myoutput.x32[position_y];
    y[32]=myoutput.x33[position_y];
    y[33]=myoutput.x34[position_y];
    y[34]=myoutput.x35[position_y];
    y[35]=myoutput.x36[position_y];
    y[36]=myoutput.x37[position_y];
    y[37]=myoutput.x38[position_y];
    y[38]=myoutput.x39[position_y];
    y[39]=myoutput.x40[position_y];
    


    
    vy[0]=myoutput.x01[velocity_y];
    vy[1]=myoutput.x02[velocity_y];
    vy[2]=myoutput.x03[velocity_y];
    vy[3]=myoutput.x04[velocity_y];
    vy[4]=myoutput.x05[velocity_y];
    vy[5]=myoutput.x06[velocity_y];
    vy[6]=myoutput.x07[velocity_y];
    vy[7]=myoutput.x08[velocity_y];
    vy[8]=myoutput.x09[velocity_y];
    vy[9]=myoutput.x10[velocity_y];
    vy[10]=myoutput.x11[velocity_y];
    vy[11]=myoutput.x12[velocity_y];
    vy[12]=myoutput.x13[velocity_y];
    vy[13]=myoutput.x14[velocity_y];
    vy[14]=myoutput.x15[velocity_y];
    vy[15]=myoutput.x16[velocity_y];
    vy[16]=myoutput.x17[velocity_y];
    vy[17]=myoutput.x18[velocity_y];
    vy[18]=myoutput.x19[velocity_y];
    vy[19]=myoutput.x20[velocity_y];
    vy[20]=myoutput.x21[velocity_y];
    vy[21]=myoutput.x22[velocity_y];
    vy[22]=myoutput.x23[velocity_y];
    vy[23]=myoutput.x24[velocity_y];
    vy[24]=myoutput.x25[velocity_y];
    vy[25]=myoutput.x26[velocity_y];
    vy[26]=myoutput.x27[velocity_y];
    vy[27]=myoutput.x28[velocity_y];
    vy[28]=myoutput.x29[velocity_y];
    vy[29]=myoutput.x30[velocity_y];
    vy[30]=myoutput.x31[velocity_y];
    vy[31]=myoutput.x32[velocity_y];
    vy[32]=myoutput.x33[velocity_y];
    vy[33]=myoutput.x34[velocity_y];
    vy[34]=myoutput.x35[velocity_y];
    vy[35]=myoutput.x36[velocity_y];
    vy[36]=myoutput.x37[velocity_y];
    vy[37]=myoutput.x38[velocity_y];
    vy[38]=myoutput.x39[velocity_y];
    vy[39]=myoutput.x40[velocity_y];
    
    vz[0]=myoutput.x01[velocity_z];
    vz[1]=myoutput.x02[velocity_z];
    vz[2]=myoutput.x03[velocity_z];
    vz[3]=myoutput.x04[velocity_z];
    vz[4]=myoutput.x05[velocity_z];
    vz[5]=myoutput.x06[velocity_z];
    vz[6]=myoutput.x07[velocity_z];
    vz[7]=myoutput.x08[velocity_z];
    vz[8]=myoutput.x09[velocity_z];
    vz[9]=myoutput.x10[velocity_z];
    vz[10]=myoutput.x11[velocity_z];
    vz[11]=myoutput.x12[velocity_z];
    vz[12]=myoutput.x13[velocity_z];
    vz[13]=myoutput.x14[velocity_z];
    vz[14]=myoutput.x15[velocity_z];
    vz[15]=myoutput.x16[velocity_z];
    vz[16]=myoutput.x17[velocity_z];
    vz[17]=myoutput.x18[velocity_z];
    vz[18]=myoutput.x19[velocity_z];
    vz[19]=myoutput.x20[velocity_z];
    vz[20]=myoutput.x21[velocity_z];
    vz[21]=myoutput.x22[velocity_z];
    vz[22]=myoutput.x23[velocity_z];
    vz[23]=myoutput.x24[velocity_z];
    vz[24]=myoutput.x25[velocity_z];
    vz[25]=myoutput.x26[velocity_z];
    vz[26]=myoutput.x27[velocity_z];
    vz[27]=myoutput.x28[velocity_z];
    vz[28]=myoutput.x29[velocity_z];
    vz[29]=myoutput.x30[velocity_z];
    vz[30]=myoutput.x31[velocity_z];
    vz[31]=myoutput.x32[velocity_z];
    vz[32]=myoutput.x33[velocity_z];
    vz[33]=myoutput.x34[velocity_z];
    vz[34]=myoutput.x35[velocity_z];
    vz[35]=myoutput.x36[velocity_z];
    vz[36]=myoutput.x37[velocity_z];
    vz[37]=myoutput.x38[velocity_z];
    vz[38]=myoutput.x39[velocity_z];
    vz[39]=myoutput.x40[velocity_z];

  
    
    z[0]=uavs_pose[drone_id].pose.pose.position.z;
    z[1]=uavs_pose[drone_id].pose.pose.position.z;
    z[2]=uavs_pose[drone_id].pose.pose.position.z;
    z[3]=uavs_pose[drone_id].pose.pose.position.z;
    z[4]=uavs_pose[drone_id].pose.pose.position.z;
    z[5]=uavs_pose[drone_id].pose.pose.position.z;
    z[6]=uavs_pose[drone_id].pose.pose.position.z;
    z[7]=uavs_pose[drone_id].pose.pose.position.z;
    z[8]=uavs_pose[drone_id].pose.pose.position.z;
    z[9]=uavs_pose[drone_id].pose.pose.position.z;
    z[10]=uavs_pose[drone_id].pose.pose.position.z;
    z[11]=uavs_pose[drone_id].pose.pose.position.z;
    z[12]=uavs_pose[drone_id].pose.pose.position.z;
    z[13]=uavs_pose[drone_id].pose.pose.position.z;
    z[14]=uavs_pose[drone_id].pose.pose.position.z;
    z[15]=uavs_pose[drone_id].pose.pose.position.z;
    z[16]=uavs_pose[drone_id].pose.pose.position.z;
    z[17]=uavs_pose[drone_id].pose.pose.position.z;
    z[18]=uavs_pose[drone_id].pose.pose.position.z;
    z[19]=uavs_pose[drone_id].pose.pose.position.z;
    z[20]=uavs_pose[drone_id].pose.pose.position.z;
    z[21]=uavs_pose[drone_id].pose.pose.position.z;
    z[22]=uavs_pose[drone_id].pose.pose.position.z;
    z[23]=uavs_pose[drone_id].pose.pose.position.z;
    z[24]=uavs_pose[drone_id].pose.pose.position.z;
    z[25]=uavs_pose[drone_id].pose.pose.position.z;
    z[26]=uavs_pose[drone_id].pose.pose.position.z;
    z[27]=uavs_pose[drone_id].pose.pose.position.z;
    z[28]=uavs_pose[drone_id].pose.pose.position.z;
    z[29]=uavs_pose[drone_id].pose.pose.position.z;
    z[30]=uavs_pose[drone_id].pose.pose.position.z;
    z[31]=uavs_pose[drone_id].pose.pose.position.z;
    z[32]=uavs_pose[drone_id].pose.pose.position.z;
    z[33]=uavs_pose[drone_id].pose.pose.position.z;
    z[34]=uavs_pose[drone_id].pose.pose.position.z;
    z[35]=uavs_pose[drone_id].pose.pose.position.z;
    z[36]=uavs_pose[drone_id].pose.pose.position.z;
    z[37]=uavs_pose[drone_id].pose.pose.position.z;
    z[38]=uavs_pose[drone_id].pose.pose.position.z;
    z[39]=uavs_pose[drone_id].pose.pose.position.z;

    
    return exitflag;

    
}







