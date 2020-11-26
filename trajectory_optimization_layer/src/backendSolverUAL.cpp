#include<backendSolverUAL.h>


void backendSolverUAL::ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {

  uavs_pose_[drone_id_].state.velocity.x = msg->twist.linear.x;
  uavs_pose_[drone_id_].state.velocity.y = msg->twist.linear.y;
  uavs_pose_[drone_id_].state.velocity.z = msg->twist.linear.z;
}

backendSolverUAL::backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh, const int time_horizon) : backendSolver::backendSolver(_pnh, _nh, time_horizon) {
  sub_velocity  = _nh.subscribe<geometry_msgs::TwistStamped>("ual/velocity", 1, &backendSolverUAL::ownVelocityCallback, this);
  // TODO include or not calls to the uav from this node
  // std::string target_topic;
  // ros::Subscriber target_pose_sub = _pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallbackGRVC);
  // main loop

  // TODO integrate it into the class UAL interface
  // pose and trajectory subscriptions
  /** for(int i=0; i<drones.size(); i++){ // for each drone, subscribe to the calculated trajectory and the drone pose
       drone_pose_sub[drones[i]] = pnh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drones[i])+"/ual/pose", 10, std::bind(&uavPoseCallback,
   std::placeholders::_1, drones[i]));               // Change for each drone ID if(drones[i] !=drone_id_){ drone_trajectory_sub[drones[i]] =
   pnh.subscribe<optimal_control_interface::Solver>("/drone_"+std::to_string(drones[i])+"/solver", 1, std::bind(&uavTrajectoryCallback, std::placeholders::_1,
   drones[i]));
       }
       //initialize
       trajectory_solved_received[drones[i]] = false;
   }*/
}

/** \brief Callback for the target pose
 */
void backendSolverUAL::targetPoseCallbackGRVC(const nav_msgs::Odometry::ConstPtr &msg) {
  target_has_pose          = true;
  target_odometry_.pose.pose = msg->pose.pose;
}

