#include<utils.h>
#include<backendSolver.h>
#include <numericalSolver.h>


SolverUtils::Logger::Logger(backendSolver* class_to_log): class_to_log_ptr_(class_to_log){

    // current time to string
    std::time_t t = std::time(nullptr);
    char str_time[80];
    std::strftime(str_time, sizeof(str_time),"%c", std::localtime(&t));
    std::string string_time(str_time);
    std::cout<<str_time<<std::endl;
    std::string mypackage = ros::package::getPath("optimal_control_interface");
    // file open
    file_.open(mypackage+ + "/logs/"+ string_time+"_drone"+std::to_string(class_to_log_ptr_->drone_id_));
    // file_ << std::fixed << std::setprecision(5);


}

SolverUtils::Logger::~Logger(){

    file_.close();

}


void SolverUtils::Logger::logging() {
  // logging all results
  file_ << "shot type " << class_to_log_ptr_->desired_type_ << std::endl;
  file_ << "Desired pose: " << class_to_log_ptr_->desired_odometry_.pose.pose.position.x << ", " << class_to_log_ptr_->desired_odometry_.pose.pose.position.y << ", "
           << class_to_log_ptr_->desired_odometry_.pose.pose.position.z << std::endl;
  file_ << "target pose: " << class_to_log_ptr_->target_odometry_.pose.pose.position.x << ", " << class_to_log_ptr_->target_odometry_.pose.pose.position.y << ", "
           << class_to_log_ptr_->target_odometry_.pose.pose.position.z << std::endl;
  file_ << "target vel: " << class_to_log_ptr_->target_odometry_.twist.twist.linear.x << ", " << class_to_log_ptr_->target_odometry_.twist.twist.linear.y << ", "
           << class_to_log_ptr_->target_odometry_.twist.twist.linear.z << std::endl;
  file_ << "My accel: " << 0.0 << ", " << 0.0 << ", " << 0.0 << std::endl;
  // logging inter-uavss pose
  for(auto it = class_to_log_ptr_->uavs_pose_.begin(); it!=class_to_log_ptr_->uavs_pose_.end();++it){
    file_ << "Drone pose "<<it->first<<": " << it->second.pose.x << ", " << it->second.pose.y << ", " << it->second.pose.z<< ", " << std::endl;
    file_ << "Drone vel "<<it->first<<": " << it->second.velocity.x << ", " << it->second.velocity.y << ", " << it->second.velocity.z<< ", " << std::endl;
  }
  file_ << "initial guess: " << std::endl;

  for (int i = 0; i <class_to_log_ptr_->initial_guess_.begin()->second.size() ; i++) {
    file_ << class_to_log_ptr_->initial_guess_["ax"][i] << ", " << class_to_log_ptr_->initial_guess_["ay"][i] << ", " << class_to_log_ptr_->initial_guess_["az"][i] << ", " << class_to_log_ptr_->initial_guess_["px"][i] << ", "
             << class_to_log_ptr_->initial_guess_["py"][i] << ", " << class_to_log_ptr_->initial_guess_["pz"][i] << ", " << class_to_log_ptr_->initial_guess_["vx"][i] << ", " << class_to_log_ptr_->initial_guess_["vy"][i] << ", "
             << class_to_log_ptr_->initial_guess_["vz"][i] << std::endl;
  }
}


void SolverUtils::Logger::loggingCalculatedTrajectory(const int solver_success) {
  file_ << "solver_success: " << solver_success << std::endl;
  file_ << "Calculated trajectroy" << std::endl;
  for (int i = 0; i < class_to_log_ptr_->time_horizon_; i++) {
    file_ << class_to_log_ptr_->solver_pt_->ax_ptr_[i] << ", " << class_to_log_ptr_->solver_pt_->ay_ptr_[i] << ", " << class_to_log_ptr_->solver_pt_->az_ptr_[i]
      << ", " << class_to_log_ptr_->solver_pt_->x_ptr_[i] << ", " << class_to_log_ptr_->solver_pt_->y_ptr_[i] << ", " << class_to_log_ptr_->solver_pt_->z_ptr_[i]
     << ", " << class_to_log_ptr_->solver_pt_->vx_ptr_[i] << ", " << class_to_log_ptr_->solver_pt_->vy_ptr_[i] << ", "<< class_to_log_ptr_->solver_pt_->vz_ptr_[i] << std::endl;
  }
}
