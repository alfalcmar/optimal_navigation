#include <backendSolver.h>
#include <iostream>

backendSolver::backendSolver(ros::NodeHandle pnh, ros::NodeHandle nh) {
  ROS_INFO("backend solver constructor");

  // drones param
  if (ros::param::has("~drones")) {
    if (!ros::param::get("~drones", drones)) {
      ROS_ERROR("'Drones' does not have the rigth type");
    }
  } else {
    ROS_ERROR("fail to get the drones ids");
  }
  // no fly zone param
  if (ros::param::has("~no_fly_zone")) {
    if (!ros::param::get("~no_fly_zone", no_fly_zone_center_)) {
      ROS_ERROR("'No fly zone param does not have the right type");
    }
  } else {
    ROS_ERROR("fail to get no fly zone param");
  }
  if (ros::param::has("~drone_id")) {
    ros::param::get("~drone_id", drone_id_);
  }
  if (ros::param::has("~trajectory_frame")) {
    ros::param::get("~trajectory_frame", trajectory_frame_);
  } else {
    ROS_ERROR("fail to get the drones id");
  }
  if (ros::param::has("~solver_rate")) {
    ros::param::get("~solver_rate", solver_rate_);
  } else {
    ROS_ERROR("fail to get solver rate");
  }
  // parameters
  for (int i = 0; i < drones.size(); i++) {
    has_poses[TARGET]    = false;
    has_poses[drones[i]] = false;
  }
  if (no_fly_zone_center_.size() == 2) {
    calculateNoFlyZonePoints(no_fly_zone_center_[0], no_fly_zone_center_[1], NO_FLY_ZONE_RADIUS);
  } else {
    ROS_ERROR("No fly zone is not set");
  }
  // subscripions
  desired_pose_sub = nh.subscribe<shot_executer::DesiredShot>("desired_pose", 1, &backendSolver::desiredPoseCallback, this);  // desired pose from shot executer
  // publishers
  desired_pose_publisher = pnh.advertise<geometry_msgs::PointStamped>("desired_point", 1);
  solved_trajectory_pub  = pnh.advertise<optimal_control_interface::Solver>("trajectory", 1);
  path_rviz_pub          = pnh.advertise<nav_msgs::Path>("path", 1);
  path_no_fly_zone       = pnh.advertise<nav_msgs::Path>("noflyzone", 1);
  target_path_rviz_pub   = pnh.advertise<nav_msgs::Path>("target/path", 1);

  // acado object and thread
  acado_solver_pt_ = new ACADOsolver(solver_rate_);
  main_thread_     = std::thread(&backendSolver::stateMachine, this);

  // log files
  std::string mypackage = ros::package::getPath("optimal_control_interface");
  csv_pose.open(mypackage + "/logs/" + "trajectories_" + std::to_string(drone_id_) + ".csv");
  csv_pose << std::fixed << std::setprecision(5);
}


void backendSolver::desiredPoseCallback(const shot_executer::DesiredShot::ConstPtr &msg) {

  desired_odometry_ = msg->desired_odometry;
  desired_type_     = msg->type;
  // ROS_INFO("Desired pose received: x: %f y: %f z: %f",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z]);
}

/** \brief uav odometry callback (mrs system)
 */
void backendSolverMRS::uavCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  uavs_pose_[drone_id_] = *msg;

  has_poses[drone_id_] = true;
}

/** \brief This callback receives the solved trajectory of uavs
 */
void backendSolver::uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id) {
  uavs_trajectory[id].positions.clear();
  uavs_trajectory[id].velocities.clear();
  uavs_trajectory[id].accelerations.clear();
  trajectory_solved_received[id]    = true;
  uavs_trajectory[id].positions     = msg->positions;
  uavs_trajectory[id].velocities    = msg->velocities;
  uavs_trajectory[id].accelerations = msg->accelerations;
  ROS_INFO("Solver %d: trajectory callback from drone %d", drone_id_, id);
}

/** \brief callback for the pose of uavs
 */
void backendSolver::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  has_poses[id] = true;
  if (!trajectory_solved_received[id]) {
    for (int i = 0; i < time_horizon_; i++) {
      geometry_msgs::PoseStamped pose_aux;
      pose_aux.pose.position.x = msg->pose.position.x;
      pose_aux.pose.position.y = msg->pose.position.y;
      pose_aux.pose.position.z = msg->pose.position.z;
      uavs_trajectory[id].positions.push_back(pose_aux);
    }
  }
  uavs_pose_[id].pose.pose = msg->pose;
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
    has_poses[TARGET]                     = true;
  }
  /* target_odometry_ = *_msg; */
}

void backendSolverMRS::publishTargetOdometry() {
  if (!has_poses[TARGET]) {
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

void backendSolver::publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int delayed_points /*0 default */) {
  ROS_INFO("virtual definition of publish solved trajectory");
}

void backendSolver::publishTrajectory() {

  optimal_control_interface::Solver traj;
  geometry_msgs::PoseStamped        pos;
  geometry_msgs::Twist              vel;

  for (int i = 0; i < time_horizon_; i++) {
    // trajectory to visualize
    pos.pose.position.x = x_[i];
    pos.pose.position.y = y_[i];
    pos.pose.position.z = z_[i];
    traj.positions.push_back(pos);
    vel.linear.x = vx_[i];
    vel.linear.y = vy_[i];
    vel.linear.z = vz_[i];
    traj.velocities.push_back(vel);
  }
  solved_trajectory_pub.publish(traj);
}


bool backendSolver::desiredPoseReached(const std::vector<double> desired_pos, const std::vector<double> last_traj_pos) {
  Eigen::Vector3f desired_pose = Eigen::Vector3f(desired_pos[0], desired_pos[1], desired_pos[2]);
  Eigen::Vector3f final_pose   = Eigen::Vector3f(last_traj_pos[0], last_traj_pos[1], last_traj_pos[2]);
  if ((desired_pose - final_pose).norm() < REACHING_TOLERANCE) {
    return true;
  } else {
    return false;
  }
}
std::vector<double> backendSolver::predictingPitch() {

  std::vector<double> pitch;

  Eigen::Vector3f target_pose_aux;
  Eigen::Vector3f drone_pose_aux;
  Eigen::Vector3f q_camera_target;
  for (int i = 0; i < time_horizon_; i++) {
    target_pose_aux = Eigen::Vector3f(target_trajectory_[i].pose.pose.position.x, target_trajectory_[i].pose.pose.position.y, 0);
    drone_pose_aux  = Eigen::Vector3f(x_[i], y_[i], z_[i]);
    q_camera_target = drone_pose_aux - target_pose_aux;
    float aux_sqrt  = sqrt(pow(q_camera_target[0], 2.0) + pow(q_camera_target[1], 2.0));
    pitch.push_back(1.57 - atan2(aux_sqrt, q_camera_target[2]));
  }
  return pitch;
}

/** utility function to move yaw pointing the target*/
std::vector<double> backendSolver::predictingYaw() {

  std::vector<double> yaw;

  Eigen::Vector3f target_pose_aux;
  Eigen::Vector3f drone_pose_aux;
  Eigen::Vector3f q_camera_target;
  for (int i = 0; i < time_horizon_; i++) {
    target_pose_aux =
        Eigen::Vector3f(target_trajectory_[i].pose.pose.position.x, target_trajectory_[i].pose.pose.position.y, target_trajectory_[i].pose.pose.position.z);
    drone_pose_aux  = Eigen::Vector3f(x_[i], y_[i], z_[i]);
    q_camera_target = target_pose_aux - drone_pose_aux;
    yaw.push_back(atan2(q_camera_target[1], q_camera_target[0]));
    ROS_INFO("[%s]: Estimated target trajectory target = [%.2f, %.2f, %.2f], drone = [%.2f, %.2f, %.2f], yaw = %.2f", ros::this_node::getName().c_str(),
             target_pose_aux[0], target_pose_aux[1], target_pose_aux[2], drone_pose_aux[0], drone_pose_aux[1], drone_pose_aux[2],
             atan2(-q_camera_target[1], -q_camera_target[0]));
  }
  return yaw;
}

void backendSolver::pruebaDroneSubida() {

  double aux = 0.0;
  double vel = 0.5;
  for (int i = 0; i < time_horizon_; i++) {
    double aux = uavs_pose_[drone_id_].pose.pose.position.z + step_size * i * vel;
    z_[i]      = aux;
  }
}

void backendSolver::targetTrajectoryVelocityCTEModel() {

  target_trajectory_.clear();
  double             target_vel_module = sqrt(pow(target_odometry_.twist.twist.linear.x, 2) + pow(target_odometry_.twist.twist.linear.y, 2));
  nav_msgs::Odometry aux;
  for (int i = 0; i < time_horizon_; i++) {
    aux.pose.pose.position.x = target_odometry_.pose.pose.position.x + step_size * i * target_odometry_.twist.twist.linear.x;
    aux.pose.pose.position.y = target_odometry_.pose.pose.position.y + step_size * i * target_odometry_.twist.twist.linear.y;
    aux.pose.pose.position.z = target_odometry_.pose.pose.position.z + step_size * i * target_odometry_.twist.twist.linear.z;
    aux.twist                = target_odometry_.twist;  // velocity constant model
    target_trajectory_.push_back(aux);
    // ROS_INFO("[%s]: Target trajectory velocity model: target = [%.2f, %.2f, %.2f],", ros::this_node::getName().c_str(), aux.pose.pose.position.x,
            // aux.pose.pose.position.y, aux.pose.pose.position.z);
  }
}

geometry_msgs::Quaternion backendSolver::toQuaternion(const double pitch, const double roll, const double yaw) {
  geometry_msgs::Quaternion q;
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;
  return q;
}

/**  \brief Construct a nav_msgs_path and publish to visualize through rviz
 *   \param wps_x, wps_y, wps_z     last calculated path
 */

void backendSolver::publishPath() {
  nav_msgs::Path                          msg;
  std::vector<geometry_msgs::PoseStamped> poses(time_horizon_);
  msg.header.frame_id = trajectory_frame_;
  for (int i = 0; i < time_horizon_; i++) {
    poses.at(i).pose.position.x    = x_[i];
    poses.at(i).pose.position.y    = y_[i];
    poses.at(i).pose.position.z    = z_[i];
    poses.at(i).pose.orientation.x = 0;
    poses.at(i).pose.orientation.y = 0;
    poses.at(i).pose.orientation.z = 0;
    poses.at(i).pose.orientation.w = 1;
  }
  msg.poses = poses;
  path_rviz_pub.publish(msg);
}


/** \brief this function publish the desired pose in a ros point type
 *  \param x,y,z        Desired pose
 */
void backendSolver::publishDesiredPoint() {
  geometry_msgs::PointStamped desired_point;
  desired_point.point.x         = desired_odometry_.pose.pose.position.x;
  desired_point.point.y         = desired_odometry_.pose.pose.position.y;
  desired_point.point.z         = desired_odometry_.pose.pose.position.z;
  desired_point.header.frame_id = trajectory_frame_;

  desired_pose_publisher.publish(desired_point);
}

/** \brief Construct the no fly zone approximated by a tetrahedron to visualize it on RVIZ
 *  \param  2D points
 */
void backendSolver::publishNoFlyZone(double point_1[2], double point_2[2], double point_3[2], double point_4[2]) {
  nav_msgs::Path msg;
  msg.header.frame_id = trajectory_frame_;

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

void backendSolver::logToCSVCalculatedTrajectory(int solver_success) {
  csv_pose << "hovering: " << hovering_ << std::endl;
  csv_pose << "first time solving: " << first_time_solving_ << std::endl;
  csv_pose << "solver_success: " << solver_success << std::endl;
  csv_pose << "Calculated trajectroy" << std::endl;
  for (int i = 0; i < time_horizon_; i++) {
    csv_pose << ax_[i] << ", " << ay_[i] << ", " << az_[i] << ", " << x_[i] << ", " << y_[i] << ", " << z_[i] << ", " << vx_[i] << ", " << vy_[i] << ", "
             << vz_[i] << std::endl;
  }
}

void backendSolver::logToCsv() {
  // logging all results
  csv_pose << "shot type " << desired_type_ << std::endl;
  csv_pose << "Desired pose: " << desired_odometry_.pose.pose.position.x << ", " << desired_odometry_.pose.pose.position.y << ", "
           << desired_odometry_.pose.pose.position.z << std::endl;
  csv_pose << "target pose: " << target_odometry_.pose.pose.position.x << ", " << target_odometry_.pose.pose.position.y << ", "
           << target_odometry_.pose.pose.position.z << std::endl;
  csv_pose << "target vel: " << target_odometry_.twist.twist.linear.x << ", " << target_odometry_.twist.twist.linear.y << ", "
           << target_odometry_.twist.twist.linear.z << std::endl;
  csv_pose << "Time horizon" << time_horizon_ << std::endl;
  csv_pose << "My pose: " << uavs_pose_[drone_id_].pose.pose.position.x << ", " << uavs_pose_[drone_id_].pose.pose.position.y << ", "
           << uavs_pose_[drone_id_].pose.pose.position.z << std::endl;
  csv_pose << "My vel: " << uavs_pose_[drone_id_].twist.twist.linear.x << ", " << uavs_pose_[drone_id_].twist.twist.linear.y << ", "
           << uavs_pose_[drone_id_].twist.twist.linear.z << std::endl;
  csv_pose << "My accel: " << 0.0 << ", " << 0.0 << ", " << 0.0 << std::endl;
  // logging inter-uavs pose
  if (multi_) {
    csv_pose << "Drone 2: " << uavs_pose_[2].pose.pose.position.x << ", " << uavs_pose_[2].pose.pose.position.y << ", " << uavs_pose_[2].pose.pose.position.z
             << ", " << std::endl;
    csv_pose << "Drone 3: " << uavs_pose_[3].pose.pose.position.x << ", " << uavs_pose_[3].pose.pose.position.y << ", " << uavs_pose_[3].pose.pose.position.z
             << ", " << std::endl;
  }
  csv_pose << "initial guess: " << std::endl;
  for (int i = 0; i < time_horizon_; i++) {
    csv_pose << initial_guess_["ax"][i] << ", " << initial_guess_["ay"][i] << ", " << initial_guess_["az"][i] << ", " << initial_guess_["px"][i] << ", "
             << initial_guess_["py"][i] << ", " << initial_guess_["pz"][i] << ", " << initial_guess_["vx"][i] << ", " << initial_guess_["vy"][i] << ", "
             << initial_guess_["vz"][i] << std::endl;
  }
  // csv_pose<<"Calculated trajectroy"<<std::endl;
  //  for(int i=0; i<x_.size(); i++){
  //     csv_pose <<ax_[i] << ", " << ay_[i] << ", " << az_[i]<< ", "<<x_[i] << ", " << y_[i] << ", " << z_[i]<< ", "<< vx_[i]<< ", " <<vy_[i]<< ", "
  //     <<vz_[i]<<std::endl;
  // }
}

nav_msgs::Path backendSolver::targetPathVisualization() {
  nav_msgs::Path                          msg;
  std::vector<geometry_msgs::PoseStamped> poses(target_trajectory_.size());
  msg.header.frame_id = trajectory_frame_;
  for (size_t i = 0; i < target_trajectory_.size(); i++) {
    poses.at(i).pose.position.x    = target_trajectory_[i].pose.pose.position.x;
    poses.at(i).pose.position.y    = target_trajectory_[i].pose.pose.position.y;
    poses.at(i).pose.position.z    = target_trajectory_[i].pose.pose.position.z;
    poses.at(i).pose.orientation.x = 0;
    poses.at(i).pose.orientation.y = 0;
    poses.at(i).pose.orientation.z = 0;
    poses.at(i).pose.orientation.w = 1;
  }
  return msg;
}

bool backendSolver::checkConnectivity() {
  // check the connectivity with drones and target
  size_t cont = 0;
  for (size_t i = 0; i < drones.size(); i++) {  // check drones pose
    if (has_poses[drones[i]] == true) {
      cont++;
    }
  }
  if (target_) {              // if target included
    if (has_poses[TARGET]) {  // check target pose
      cont++;
    }
    if (cont == drones.size() + 1) {  //+1 because the target
      return true;
    } else {
      return false;
    }
  } else {
    if (cont == drones.size()) {
      return true;
    } else {
      return false;
    }
  }
}

void backendSolver::calculateNoFlyZonePoints(const float x_center, const float y_center, const float radius) {
  no_fly_zone_points_.clear();
  std::array<float, 2> aux;
  const float          SAFETY_OFFSET = 4;
  for (float angle = 0.0; angle < 2 * M_PI; angle = angle + 0.25) {
    aux[0] = x_center + (radius + SAFETY_OFFSET) * cos(angle);
    aux[1] = y_center + (radius + SAFETY_OFFSET) * sin(angle);
    no_fly_zone_points_.push_back(aux);
  }
}

int backendSolver::closestPose() {
  int   nearest_point    = 0;
  float nearest_distance = INFINITY;
  float point_distance   = 0;
  for (int i = 0; i < time_horizon_; i++) {
    point_distance = sqrt(pow((x_[i] - uavs_pose_[drone_id_].pose.pose.position.x), 2) + pow((y_[i] - uavs_pose_[drone_id_].pose.pose.position.y), 2) +
                          pow((z_[i] - uavs_pose_[drone_id_].pose.pose.position.z), 2));
    if (point_distance < nearest_distance) {
      nearest_distance = point_distance;
      nearest_point    = i;
    }
  }
  return nearest_point;
}

bool backendSolver::isDesiredPoseReached(const nav_msgs::Odometry &_desired_pose, const nav_msgs::Odometry &_last_pose) {
  ROS_INFO("Desired pose reached");
}

void backendSolver::calculateInitialGuess(bool new_initial_guess) {
  csv_pose<<"change initial guess: "<<new_initial_guess<<std::endl;
  if (new_initial_guess) {
    std::array<float, 2> aux;
    // calculate scalar direction
    float aux_norm       = sqrt(pow((desired_odometry_.pose.pose.position.x - uavs_pose_[drone_id_].pose.pose.position.x), 2) +
                          pow((desired_odometry_.pose.pose.position.y - uavs_pose_[drone_id_].pose.pose.position.y), 2) +
                          pow((desired_odometry_.pose.pose.position.z - uavs_pose_[drone_id_].pose.pose.position.z), 2));
    float scalar_dir_x   = (desired_odometry_.pose.pose.position.x - uavs_pose_[drone_id_].pose.pose.position.x) / aux_norm;
    float scalar_dir_y   = (desired_odometry_.pose.pose.position.y - uavs_pose_[drone_id_].pose.pose.position.y) / aux_norm;
    float scalar_dir_z   = (desired_odometry_.pose.pose.position.z - uavs_pose_[drone_id_].pose.pose.position.z) / aux_norm;
    float vel_module_cte = max_vel / 2;  // vel cte guess for the initial
    // accelerations
    initial_guess_["ax"][0] = ZERO;
    initial_guess_["ay"][0] = ZERO;
    initial_guess_["az"][0] = ZERO;
    initial_guess_["px"][0] = uavs_pose_[drone_id_].pose.pose.position.x;
    initial_guess_["py"][0] = uavs_pose_[drone_id_].pose.pose.position.y;
    initial_guess_["pz"][0] = uavs_pose_[drone_id_].pose.pose.position.z;
    initial_guess_["vx"][0] = scalar_dir_x * vel_module_cte;
    initial_guess_["vy"][0] = scalar_dir_y * vel_module_cte;
    initial_guess_["vz"][0] = scalar_dir_z * vel_module_cte;
    for (int i = 1; i < time_horizon_; i++) {
      initial_guess_["ax"][i] = ZERO;
      initial_guess_["ay"][i] = ZERO;
      initial_guess_["az"][i] = ZERO;
      initial_guess_["vx"][i] = scalar_dir_x * vel_module_cte;
      initial_guess_["vy"][i] = scalar_dir_y * vel_module_cte;
      initial_guess_["vz"][i] = scalar_dir_z * vel_module_cte;
      initial_guess_["px"][i] = initial_guess_["px"][i - 1] + step_size * initial_guess_["vx"][i - 1];
      initial_guess_["py"][i] = initial_guess_["py"][i - 1] + step_size * initial_guess_["vy"][i - 1];
      initial_guess_["pz"][i] = initial_guess_["pz"][i - 1] + step_size * initial_guess_["vz"][i - 1];
      // no fly zone
      if (pow(initial_guess_["px"][i] - no_fly_zone_center_[0], 2) + pow(initial_guess_["py"][i] - no_fly_zone_center_[1], 2) < pow(NO_FLY_ZONE_RADIUS, 2)) {
        csv_pose << "expanding pose " << i << std::endl;
        aux                     = expandPose(initial_guess_["px"][i], initial_guess_["py"][i]);
        initial_guess_["px"][i] = aux[0];
        initial_guess_["py"][i] = aux[1];
      }
    }
  } else {
    // previous one
    for (int i = 0; i < time_horizon_; i++) {
      initial_guess_["ax"][i] = ax_[i];
      initial_guess_["ay"][i] = ay_[i];
      initial_guess_["az"][i] = az_[i];
      initial_guess_["px"][i] = x_[i];
      initial_guess_["py"][i] = y_[i];
      initial_guess_["pz"][i] = z_[i];
      initial_guess_["vx"][i] = vx_[i];
      initial_guess_["vy"][i] = vy_[i];
      initial_guess_["vz"][i] = vz_[i];
    }
  }
  for (int i = 0; i < time_horizon_; i++) {
      initial_guess_["pitch"][i] = 0.3;
  }
  // log the initial guess
  logToCsv();
}

std::array<float, 2> backendSolver::expandPose(float x, float y) {
  float                aux_norm       = 0.0;
  float                minor_distance = INFINITY;
  std::array<float, 2> point_to_return;

  for (int i = 0; i < no_fly_zone_points_.size(); i++) {
    aux_norm = sqrt(pow((no_fly_zone_points_[i][0] - x), 2) + pow((no_fly_zone_points_[i][1] - y), 2));
    if (aux_norm < minor_distance) {
      minor_distance     = aux_norm;
      point_to_return[0] = no_fly_zone_points_[i][0];
      point_to_return[1] = no_fly_zone_points_[i][1];
    }
  }
  return point_to_return;
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

void backendSolver::staticLoop() {
  for (int i = 0; i < time_horizon_; i++) {  // maintain the position
    x_[i] = uavs_pose_[drone_id_].pose.pose.position.x;
    y_[i] = uavs_pose_[drone_id_].pose.pose.position.y;
  }
  // TODO think about what to do with that
  if (subida) {
    pruebaDroneSubida();
  } else {
    for (int i = 0; i < time_horizon_; i++) {
      z_[i] = uavs_pose_[drone_id_].pose.pose.position.z;
    }
  }

  targetTrajectoryVelocityCTEModel();
  std::vector<double> yaw   = predictingYaw();
  std::vector<double> pitch = predictingPitch();


  publishSolvedTrajectory(yaw, pitch);
}

void backendSolver::IDLEState() {
  ROS_INFO("Solver %d: IDLE state", drone_id_);
}

// void backendSolver::deletingPoints(const int number_of_points){
//         x_.erase(x_.begin(),x_.begin()+number_of_points);
//         y_.erase(y_.begin(),y_.begin()+number_of_points);
//         z_.erase(z_.begin(),z_.begin()+number_of_points);
//         vx_.erase(vx_.begin(),vx_.begin()+number_of_points);
//         vy_.erase(vy_.begin(),vy_.begin()+number_of_points);
//         vz_.erase(vz_.begin(),vz_.begin()+number_of_points);
//         ax_.erase(ax_.begin(),ax_.begin()+number_of_points);
//         ay_.erase(ay_.begin(),ay_.begin()+number_of_points);
//         az_.erase(az_.begin(),az_.begin()+number_of_points);
// }
// void backendSolver::checkPoints(){
//     for(int i=0;x_.size();i++){

//     }
// }
float backendSolver::checkRoundedTime(std::chrono::system_clock::time_point start) {
  std::chrono::duration<double> diff             = std::chrono::system_clock::now() - start;
  csv_pose<<"rate: "<<diff.count()<<"(s)"<<std::endl<<std::endl<<std::endl<<std::endl;
  float rounded_time = round( diff.count() * 10.0 ) / 10.0;
  csv_pose<<"rate round: "<<rounded_time<<std::endl;
  return rounded_time;
}


void backendSolver::stateMachine() {
  int       closest_point = 0;
  ros::Rate solver(solver_rate_);
  bool      loop_rate_violated = false;
  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> diff;
  float actual_cicle_time = 0.0;
  bool change_initial_guess = true;
  // int cont =
  float time_initial_position = 0.0;
  first_time_solving_ = true;
  
  while (ros::ok) {
    solver.reset();
    time_initial_position = checkRoundedTime(start);
    start = std::chrono::system_clock::now();
    if (desired_type_ == shot_executer::DesiredShot::IDLE) { // IDLE STATE
      IDLEState();
    } else if (desired_type_ == shot_executer::DesiredShot::GOTO || desired_type_ == shot_executer::DesiredShot::SHOT) { // Shooting action
       
      // publish the last calculated trajectory if the solver successed
      if (solver_success == 0) {
        std::vector<double> yaw   = predictingYaw();
        std::vector<double> pitch = predictingPitch();
        publishSolvedTrajectory(yaw, pitch, closest_point);
      }
      do {
        ros::spinOnce();
        // predict the target trajectory if it exists
        if (target_) {  
          targetTrajectoryVelocityCTEModel();
        }
        // if it is the first time or the previous time the solver couldn't success, don't take previous trajectory as initial guess
        if(first_time_solving_ || change_initial_guess){ 
            calculateInitialGuess(change_initial_guess);
        }
        // call the solver
        solver_success = acado_solver_pt_->solverFunction(initial_guess_, ax_, ay_, az_, x_, y_, z_, vx_, vy_, vz_, desired_odometry_, no_fly_zone_center_,
                                                            target_trajectory_, uavs_pose_, time_initial_position, first_time_solving_);  // ACADO
        // solver_success = solver_.solverFunction(initial_guess_,ax_,ay_,az_,x_,y_,z_,vx_,vy_,vz_, desired_odometry_,
        // no_fly_zone_center_,target_trajectory_,uavs_pose_);   // call the solver function  FORCES_PRO.h
        
        // log solved trajectory
        logToCSVCalculatedTrajectory(solver_success);

        // if the solver didn't success, change initial guess
        if(solver_success !=0){
          change_initial_guess = true;
        }else{
          change_initial_guess = false;
        }  
      } while (solver_success != 0);
    }
  
    solver.sleep();
    // check and log the time that the last cycle lasted
    actual_cicle_time = solver.cycleTime().toSec();
    csv_pose << "cycle time: " << actual_cicle_time << std::endl;
    
    // when the solver is called, set first_time_solving to false
    if(solver_success == 0 && first_time_solving_){
      first_time_solving_ = false;
    }
  }
}


backendSolverMRS::backendSolverMRS(ros::NodeHandle &_pnh, ros::NodeHandle &_nh) : backendSolver::backendSolver(_pnh, _nh) {
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
  while (!checkConnectivity()) {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("Solver %d is not ready", drone_id_);
  }
  is_initialized = true;
  // main_thread_ = std::thread(&backendSolverMRS::stateMachine,this);

  ROS_INFO("Solver %d is ready", drone_id_);
}


void backendSolverMRS::publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int closest_point) {
  publishPath();
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
    aux_point.position.x = x_[i];
    aux_point.position.y = y_[i];
    aux_point.position.z = z_[i];
    aux_point.heading    = yaw[i];
    // trajectory to followers
    aux_point_for_followers.x     = x_[i];
    aux_point_for_followers.y     = y_[i];
    aux_point_for_followers.z     = z_[i];
    aux_point_for_followers.yaw   = yaw[i];
    aux_point_for_followers.pitch = pitch[i];
    aux_point_for_followers.phi   = 0.0;
    aux_point_for_followers.mode  = 2;

    traj_to_followers.points.push_back(aux_point_for_followers);
    traj_to_command.points.push_back(aux_point);
  }
  for (size_t k = 0; k < traj_to_followers.points.size(); k++) {
    ROS_INFO("[%s]: Traj to followers %u: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), traj_to_followers.points[k].x, traj_to_followers.points[k].y,
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

#ifdef UAL
void backendSolverUAL::ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg) {
  ual_state_.state = msg->state;
}

void backendSolverUAL::ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  uavs_pose_[drone_id_].twist.twist = msg->twist;
}

backendSolverUAL::backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh) : backendSolver::backendSolver(_pnh, _nh) {
  sub_velocity  = _nh.subscribe<geometry_msgs::TwistStamped>("ual/velocity", 1, &backendSolverUAL::ownVelocityCallback, this);
  uav_state_sub = _nh.subscribe<uav_abstraction_layer::State>("ual/state", 1, &backendSolverUAL::ualStateCallback, this);
  // TODO include or not calls to the uav from this node
  // go_to_waypoint_client = pnh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
  // take_off_srv = pnh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
  // ros::Subscriber target_pose_sub = pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallbackGRVC);
  // set_velocity_pub = pnh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
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
  has_poses[TARGET]          = true;
  target_odometry_.pose.pose = msg->pose.pose;
}

#endif
