#include<logger.h>
#include<backendSolver.h>


SolverUtils::Logger::Logger(backendSolver* class_to_log, ros::NodeHandle pnh): class_to_log_ptr_(class_to_log){

    // current time to string
    std::time_t t = std::time(nullptr);
    char str_time[80];
    std::strftime(str_time, sizeof(str_time),"%c", std::localtime(&t));
    std::string string_time(str_time);
    std::cout<<str_time<<std::endl;
    std::string mypackage = ros::package::getPath("optimal_control_interface");
    // file open
    file_.open("/home/alfonso/logs/"+ string_time+"_drone"+std::to_string(class_to_log_ptr_->drone_id_));
    // file_ << std::fixed << std::setprecision(5);

    path_rviz_pub          = pnh.advertise<nav_msgs::Path>("path", 1);
    path_no_fly_zone       = pnh.advertise<nav_msgs::Path>("noflyzone", 1);
    target_path_rviz_pub   = pnh.advertise<nav_msgs::Path>("target/path", 1);

}

SolverUtils::Logger::~Logger(){

    file_.close();

}

void SolverUtils::Logger::logTime(float time){
  file_ << "solving time: "<<time<<std::endl;
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
    file_ << "Drone pose "<<it->first<<": " << it->second.state.pose.x << ", " << it->second.state.pose.y << ", " << it->second.state.pose.z<< ", " << std::endl;
    file_ << "Drone vel "<<it->first<<": " << it->second.state.velocity.x << ", " << it->second.state.velocity.y << ", " << it->second.state.velocity.z<< ", " << std::endl;
  }
  file_ << "initial guess: " << std::endl;

  for (int i = 0; i <class_to_log_ptr_->time_horizon_ ; i++) {
    file_ << class_to_log_ptr_->initial_guess_[i].acc.x << ", " << class_to_log_ptr_->initial_guess_[i].acc.y << ", " << class_to_log_ptr_->initial_guess_[i].acc.z << ", " << class_to_log_ptr_->initial_guess_[i].pose.x<< ", "
             << class_to_log_ptr_->initial_guess_[i].pose.y << ", " << class_to_log_ptr_->initial_guess_[i].pose.z << ", " << class_to_log_ptr_->initial_guess_[i].velocity.x << ", " << class_to_log_ptr_->initial_guess_[i].velocity.y << ", "
             << class_to_log_ptr_->initial_guess_[i].velocity.z<< std::endl;
  }
}


void SolverUtils::Logger::loggingCalculatedTrajectory(const int solver_success) {
  file_ << "solver_success: " << solver_success << std::endl;
  file_ << "Calculated trajectroy" << std::endl;
  for (int i = 0; i < class_to_log_ptr_->time_horizon_; i++) {
    file_ << class_to_log_ptr_->solver_pt_->solution_[i].acc.x << ", " << class_to_log_ptr_->solver_pt_->solution_[i].acc.y << ", " << class_to_log_ptr_->solver_pt_->solution_[i].acc.z
      << ", " << class_to_log_ptr_->solver_pt_->solution_[i].pose.x << ", " << class_to_log_ptr_->solver_pt_->solution_[i].pose.y << ", " << class_to_log_ptr_->solver_pt_->solution_[i].pose.z
     << ", " << class_to_log_ptr_->solver_pt_->solution_[i].velocity.x << ", " << class_to_log_ptr_->solver_pt_->solution_[i].velocity.y << ", "<< class_to_log_ptr_->solver_pt_->solution_[i].velocity.z<< std::endl;
  }
}

/**  \brief Construct a nav_msgs_path and publish to visualize through rviz
 *   \param wps_x, wps_y, wps_z     last calculated path
 */

void SolverUtils::Logger::publishPath() {
  nav_msgs::Path                          msg;
  std::vector<geometry_msgs::PoseStamped> poses(class_to_log_ptr_->time_horizon_);
  msg.header.frame_id = class_to_log_ptr_->trajectory_frame_;
  for (int i = 0; i < class_to_log_ptr_->time_horizon_; i++) {
    poses.at(i).pose.position.x    =class_to_log_ptr_->solution_[i].pose.x;
    poses.at(i).pose.position.y    =class_to_log_ptr_->solution_[i].pose.y;
    poses.at(i).pose.position.z    =class_to_log_ptr_->solution_[i].pose.z;
    poses.at(i).pose.orientation.x = 0;
    poses.at(i).pose.orientation.y = 0;
    poses.at(i).pose.orientation.z = 0;
    poses.at(i).pose.orientation.w = 1;
  }
  msg.poses = poses;
  path_rviz_pub.publish(msg);
}

/** \brief Construct the no fly zone approximated by a tetrahedron to visualize it on RVIZ
 *  \param  2D points
 */
void SolverUtils::Logger::publishNoFlyZone(double point_1[2], double point_2[2], double point_3[2], double point_4[2]) {
  nav_msgs::Path msg;
  msg.header.frame_id = class_to_log_ptr_->trajectory_frame_;

  std::vector<geometry_msgs::PoseStamped> poses;
  geometry_msgs::PoseStamped              pose;

  pose.pose.position.x = point_1[0];
  pose.pose.position.y = point_1[1];
  poses.push_back(pose);

  pose.pose.position.x = point_2[0];
  pose.pose.position.y = point_2[1];
  poses.push_back(pose);

  pose.pose.position.x = point_3[0];
  pose.pose.position.y = point_3[1];
  poses.push_back(pose);

  pose.pose.position.x = point_4[0];
  pose.pose.position.y = point_4[1];
  poses.push_back(pose);

  pose.pose.position.x = point_1[0];
  pose.pose.position.y = point_1[1];


  poses.push_back(pose);

  msg.poses = poses;
  path_no_fly_zone.publish(msg);
}

nav_msgs::Path SolverUtils::Logger::targetPathVisualization() {
  nav_msgs::Path                          msg;
  std::vector<geometry_msgs::PoseStamped> poses(class_to_log_ptr_->target_trajectory_.size());
  msg.header.frame_id = class_to_log_ptr_->trajectory_frame_;
  for (size_t i = 0; i < class_to_log_ptr_->target_trajectory_.size(); i++) {
    poses.at(i).pose.position.x    = class_to_log_ptr_->target_trajectory_[i].pose.pose.position.x;
    poses.at(i).pose.position.y    = class_to_log_ptr_->target_trajectory_[i].pose.pose.position.y;
    poses.at(i).pose.position.z    = class_to_log_ptr_->target_trajectory_[i].pose.pose.position.z;
    poses.at(i).pose.orientation.x = 0;
    poses.at(i).pose.orientation.y = 0;
    poses.at(i).pose.orientation.z = 0;
    poses.at(i).pose.orientation.w = 1;
  }
  return msg;
}