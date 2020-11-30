#include<backendSolverUAL.h>


void backendSolverUAL::ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {

  uavs_pose_[drone_id_].state.velocity.x = msg->twist.linear.x;
  uavs_pose_[drone_id_].state.velocity.y = msg->twist.linear.y;
  uavs_pose_[drone_id_].state.velocity.z = msg->twist.linear.z;
}

backendSolverUAL::backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh, const int time_horizon) : backendSolver::backendSolver(_pnh, _nh, time_horizon) {
  // UAV state subscription
  uav_state_sub_ = _pnh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drone_id_)+"/ual/pose", 1, &backendSolverUAL::uavPoseCallback,this);       
  sub_velocity_  = _nh.subscribe<geometry_msgs::TwistStamped>("/drone_"+std::to_string(drone_id_)+"/ual/velocity", 1, &backendSolverUAL::ownVelocityCallback, this);
  // target subscription
  target_pose_sub_ = _pnh.subscribe<nav_msgs::Odometry>("target_topic", 1, &backendSolverUAL::targetPoseCallbackGRVC, this);
}

/** \brief Callback for the target pose
 */
void backendSolverUAL::targetPoseCallbackGRVC(const nav_msgs::Odometry::ConstPtr &msg) {
  target_has_pose          = true;
  target_odometry_ = *msg;
}

void backendSolverUAL::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  uavs_pose_[drone_id_].state.pose.x = msg->pose.position.x;
  uavs_pose_[drone_id_].state.pose.y = msg->pose.position.y;
  uavs_pose_[drone_id_].state.pose.z = msg->pose.position.z;

  uavs_pose_[drone_id_].state.quaternion.x = msg->pose.orientation.x;
  uavs_pose_[drone_id_].state.quaternion.y = msg->pose.orientation.y;
  uavs_pose_[drone_id_].state.quaternion.z = msg->pose.orientation.z;
  uavs_pose_[drone_id_].state.quaternion.w = msg->pose.orientation.w; 
}

