#include<backendSolverMRS.h>

backendSolverMRS::backendSolverMRS(ros::NodeHandle &_pnh, ros::NodeHandle &_nh, const int time_horizon) : backendSolver::backendSolver(_pnh, _nh, time_horizon) {
  ROS_INFO("Leader constructor");

  /* std::string target_topic; */
  /* _pnh.param<std::string>("target_topic",target_topic, "/gazebo/dynamic_model/jeff_electrician/odometry"); // target topic
   * /gazebo/dynamic_target/dynamic_pickup/pose */
  target_array_sub           = _pnh.subscribe<nav_msgs::Odometry>("target_topic", 1, &backendSolverMRS::targetCallbackMRS, this);  // target pose
  mrs_trajectory_tracker_pub = _pnh.advertise<mrs_msgs::TrajectoryReference>("desired_trajectory", 1);
  solved_trajectory_MRS_pub  = _pnh.advertise<formation_church_planning::Trajectory>("planned_trajectory", 1);
  uav_odometry_sub           = _pnh.subscribe<nav_msgs::Odometry>("odometry_in", 1, &backendSolverMRS::uavCallback, this);
  diagnostics_pub            = _pnh.advertise<formation_church_planning::Diagnostic>("diagnostics", 1);
  target_odometry_pub        = _pnh.advertise<nav_msgs::Odometry>("target_odometry_out", 1);
  mrs_status_pub             = _pnh.advertise<formation_church_planning::Status>("mrs_status", 1);
  service_for_activation     = _pnh.advertiseService("toggle_state", &backendSolverMRS::activationServiceCallback, this);

  ros::Rate rate(1);  // hz
  // publish diag message before getting the target pose, otherwise the mission controller does not let the UAv to take off since is supposes that the planning
  // node is not running
  ROS_INFO("[%s]: Register diag timer", ros::this_node::getName().c_str());
  diagnostic_timer_ = _nh.createTimer(ros::Duration(diagnostic_timer_rate_), &backendSolverMRS::diagTimer, this);
  transformer_      = mrs_lib::Transformer("optimal_control_interface", "uav44");

  is_initialized = true;

  std::cout<<ANSI_COLOR_YELLOW<<"Drone "<<drone_id_<<": connecting to others and target..."<<std::endl;
  while (!checkConnectivity()) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  int c;
  ROS_INFO("Press a key to start a mission");
  c = getchar();
}


void backendSolverMRS::publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int closest_point) {
  publishState(true);
  mrs_msgs::Reference                   aux_point;
  formation_church_planning::Point      aux_point_for_followers;
  mrs_msgs::TrajectoryReference         traj_to_command;
  formation_church_planning::Trajectory traj_to_followers;
  traj_to_command.fly_now         = true;
  traj_to_command.use_heading     = true;
  traj_to_command.header.frame_id = trajectory_frame_;
  traj_to_command.dt              = 0.2;

  // check that _x _y _z are the same size
  for (int i = closest_point; i < time_horizon_; i++) {

    // trajectory to command
    aux_point.position.x = solution_[i].pose.x;
    aux_point.position.y = solution_[i].pose.y;
    aux_point.position.z = solution_[i].pose.z;
    aux_point.heading    = yaw[i]+OFFSET_YAW;
    // trajectory to followers
    aux_point_for_followers.x     = solution_[i].pose.x;
    aux_point_for_followers.y     = solution_[i].pose.y;
    aux_point_for_followers.z     = solution_[i].pose.z;
    aux_point_for_followers.yaw   = yaw[i]+OFFSET_YAW;
    aux_point_for_followers.pitch = pitch[i];
    aux_point_for_followers.phi   = 0.0;
    aux_point_for_followers.mode  = 2;

    traj_to_followers.points.push_back(aux_point_for_followers);
    traj_to_command.points.push_back(aux_point);
  }
  for (size_t k = 0; k < traj_to_followers.points.size(); k++) {
    ROS_INFO("[%s]: Traj to followers %u: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), drone_id_, traj_to_followers.points[k].x, traj_to_followers.points[k].y,
             traj_to_followers.points[k].z);
  }
  traj_to_command.header.stamp = ros::Time::now();
  traj_to_followers.stamp      = ros::Time::now();
  publishTargetOdometry();
  solved_trajectory_MRS_pub.publish(traj_to_followers);
  mrs_trajectory_tracker_pub.publish(traj_to_command);
}

void backendSolverMRS::diagTimer(const ros::TimerEvent &event) {
  /* if (!is_initialized){ */
  /*   return; */
  /* } */
  publishTargetOdometry();
  formation_church_planning::Diagnostic diag_msg;
  diag_msg.header.stamp               = ros::Time::now();
  diag_msg.uav_name                   = "uav" + std::to_string(44);
  diag_msg.robot_role                 = "leader";
  diag_msg.state                      = "waiting_in_initial_position";
  diag_msg.flying_mode                = "1";
  diag_msg.ready                      = true;
  diag_msg.dist_from_desired_position = 0;
  diag_msg.last_time_solution_found   = ros::Time::now() - ros::Time::now();  // alternatively: ros::Duration(0, 0);
  diag_msg.planning_activated         = true;
  diag_msg.planning_finished          = true;
  try {
    ROS_INFO_ONCE("[%s]: Publishing diag message.", ros::this_node::getName().c_str());
    diagnostics_pub.publish(diag_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", diagnostics_pub.getTopic().c_str());
  }
}

void backendSolverMRS::publishState(const bool state) {
  formation_church_planning::Status msg;
  msg.ready    = state;
  msg.uav_name = "uav" + std::to_string(44);
  try {
    mrs_status_pub.publish(msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", mrs_status_pub.getTopic().c_str());
  }
}

/** \brief uav odometry callback (mrs system)
 */
void backendSolverMRS::uavCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  uavs_pose_[drone_id_].state.pose.x = msg->pose.pose.position.x;
  uavs_pose_[drone_id_].state.pose.y = msg->pose.pose.position.y;
  uavs_pose_[drone_id_].state.pose.z = msg->pose.pose.position.z;

  uavs_pose_[drone_id_].state.quaternion.x = msg->pose.pose.orientation.x;
  uavs_pose_[drone_id_].state.quaternion.y = msg->pose.pose.orientation.y;
  uavs_pose_[drone_id_].state.quaternion.z = msg->pose.pose.orientation.z;
  uavs_pose_[drone_id_].state.quaternion.w = msg->pose.pose.orientation.w;

  uavs_pose_[drone_id_].state.velocity.x = msg->twist.twist.linear.x;
  uavs_pose_[drone_id_].state.velocity.y = msg->twist.twist.linear.y;
  uavs_pose_[drone_id_].state.velocity.z = msg->twist.twist.linear.z;

  uavs_pose_[drone_id_].has_pose = true;
}



void backendSolverMRS::targetCallbackMRS(const nav_msgs::Odometry::ConstPtr &_msg) {
  transformer_.setCurrentControlFrame(trajectory_frame_);
  geometry_msgs::PoseStamped pose_tmp;
  pose_tmp.header = _msg->header;
  pose_tmp.pose   = _msg->pose.pose;
  ROS_WARN_THROTTLE(1.0, "[%s]: Target odometry local origin obtained - [%.2f, %.2f, %.2f].", ros::this_node::getName().c_str(), pose_tmp.pose.position.x,
                    pose_tmp.pose.position.y, pose_tmp.pose.position.z);
  auto response_pose = transformer_.transformSingle(trajectory_frame_, pose_tmp);
  ROS_WARN_THROTTLE(1.0, "[%s]: Target odometry transformed - [%.2f, %.2f, %.2f].", ros::this_node::getName().c_str(), response_pose->pose.position.x,
                    response_pose->pose.position.y, response_pose->pose.position.z);
  geometry_msgs::Vector3Stamped global_vel;
  global_vel.header   = _msg->header;
  global_vel.vector.x = _msg->twist.twist.linear.x;
  global_vel.vector.y = _msg->twist.twist.linear.y;
  global_vel.vector.z = _msg->twist.twist.linear.z;
  auto response_vel   = transformer_.transformSingle(trajectory_frame_, global_vel);
  if (response_pose && response_vel) {
    ROS_INFO_THROTTLE(1.0, "[%s]: Target odometry succesfully transformed", ros::this_node::getName().c_str());
    target_odometry_.pose.pose            = response_pose.value().pose;
    target_odometry_.twist.twist.linear.x = response_vel.value().vector.x;
    target_odometry_.twist.twist.linear.y = response_vel.value().vector.y;
    target_odometry_.twist.twist.linear.z = response_vel.value().vector.z;
    target_has_pose                     = true;
  }
  /* target_odometry_ = *_msg; */
}

void backendSolverMRS::publishTargetOdometry() {
  if (!target_has_pose) {
    return;
  }
  nav_msgs::Odometry msg = target_odometry_;
  msg.header.frame_id    = "uav47/gps_origin";
  try {
    target_odometry_pub.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", target_odometry_pub.getTopic().c_str());
  }
}

bool backendSolverMRS::activationServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation service called.", ros::this_node::getName().c_str());
  if (first_activation_) {
    ROS_INFO("[%s]: Initial pose set.", ros::this_node::getName().c_str());
    // setInitialPose();
    first_activation_ = false;
  }
  res.message = "Planning activated.";
  res.success = true;
  /* if (activated_ == req.data) { //if it was already in the state that is received */
  /*   res.success = false; */
  /*   if (req.data) { // activated */
  /*     res.message = "Planning is already activated."; */
  /*     ROS_ERROR("%s", res.message.c_str()); */
  /*   } else {    // deactivated */
  /*     res.message = "Planning is already deactivated."; */
  /*     ROS_ERROR("%s", res.message.c_str()); */
  /*   } */
  /*   return true; */
  /* } */
  /* if (req.data) { */
  /*   if (!planning_done_) { */
  /*     res.message = "Planning activated."; */
  /*     res.success = true; */
  /*     ROS_WARN("%s", res.message.c_str()); */
  /*     activated_ = req.data; */
  /*   } else { */
  /*     res.message = "Planning cannot be activated because it is finished. Call reset service."; */
  /*     res.success = false; */
  /*     ROS_ERROR("%s", res.message.c_str()); */
  /*   } */
  /* } else { */
  /*   subida = true; */
  /*   res.message = "Planning deactivated."; */
  /*   res.success = true; */
  /*   ROS_WARN("%s", res.message.c_str()); */
  /*   activated_ = req.data; */
  /* } */
}
