#include <backendSolver.h>
#include <iostream>
#include <logger.h>



backendSolver::backendSolver(ros::NodeHandle pnh, ros::NodeHandle nh, const int time_horizon) : time_horizon_(time_horizon),
                                                                                                solution_(new State[time_horizon]),
                                                                                                initial_guess_(new State[time_horizon]) {
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

  if (no_fly_zone_center_.size() == 2) {
    calculateNoFlyZonePoints(no_fly_zone_center_[0], no_fly_zone_center_[1], NO_FLY_ZONE_RADIUS);
  } else {
    ROS_ERROR("No fly zone is not set");
  }

  
  // subscripions
  desired_pose_sub = nh.subscribe<shot_executer::DesiredShot>("shot_executer_node/desired_pose", 1, &backendSolver::desiredPoseCallback, this);  // desired pose from shot executer
  // publishers
  solved_trajectory_pub  = pnh.advertise<optimal_control_interface::Solver>("trajectory", 1);

  // acado object
  solver_pt_ = std::make_unique<NumericalSolver::ACADOSolver>(solver_rate_, time_horizon, initial_guess_);

  // log files
  logger = new SolverUtils::Logger(this,pnh);
}


void backendSolver::desiredPoseCallback(const shot_executer::DesiredShot::ConstPtr &msg) {

  desired_odometry_ = msg->desired_odometry;
  desired_type_     = msg->type;
  // ROS_INFO("Desired pose received: x: %f y: %f z: %f",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z]);
}



void backendSolver::saveCalculatedTrajectory(){
  for (int i=0; i<time_horizon_; i++){
      solution_[i].pose.x = solver_pt_->solution_[i].pose.x;
      solution_[i].pose.y = solver_pt_->solution_[i].pose.y;
      solution_[i].pose.z = solver_pt_->solution_[i].pose.z;
      solution_[i].velocity.x = solver_pt_->solution_[i].velocity.x;
      solution_[i].velocity.y = solver_pt_->solution_[i].velocity.y;
      solution_[i].velocity.z = solver_pt_->solution_[i].velocity.z;
      solution_[i].acc.x = solver_pt_->solution_[i].acc.x;
      solution_[i].acc.y = solver_pt_->solution_[i].acc.y;
      solution_[i].acc.z = solver_pt_->solution_[i].acc.z;
  }
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
  uavs_pose_[id].has_pose = true;
  if (!trajectory_solved_received[id]) {
    for (int i = 0; i < time_horizon_; i++) {
      geometry_msgs::PoseStamped pose_aux;
      pose_aux.pose.position.x = msg->pose.position.x;
      pose_aux.pose.position.y = msg->pose.position.y;
      pose_aux.pose.position.z = msg->pose.position.z;
      uavs_trajectory[id].positions.push_back(pose_aux);
    }
  }
  uavs_pose_[id].state.pose.x = msg->pose.position.x;
  uavs_pose_[id].state.pose.y = msg->pose.position.y;
  uavs_pose_[id].state.pose.z = msg->pose.position.z;
  
  uavs_pose_[id].state.quaternion.x = msg->pose.orientation.x;
  uavs_pose_[id].state.quaternion.y = msg->pose.orientation.y;
  uavs_pose_[id].state.quaternion.z = msg->pose.orientation.z;
  uavs_pose_[id].state.quaternion.w = msg->pose.orientation.w;
}


void backendSolver::publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int delayed_points /*0 default */) {
  ROS_INFO("virtual definition of publish solved trajectory");
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
    drone_pose_aux  = Eigen::Vector3f(solution_[i].pose.x,solution_[i].pose.y, solution_[i].pose.z);
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
    drone_pose_aux  = Eigen::Vector3f(solution_[i].pose.x,solution_[i].pose.y, solution_[i].pose.z);
    q_camera_target = target_pose_aux - drone_pose_aux;
    yaw.push_back(atan2(q_camera_target[1], q_camera_target[0]));
    ROS_INFO("[%s]: Estimated target trajectory target = [%.2f, %.2f, %.2f], drone = [%.2f, %.2f, %.2f], yaw = %.2f", ros::this_node::getName().c_str(),
             target_pose_aux[0], target_pose_aux[1], target_pose_aux[2], drone_pose_aux[0], drone_pose_aux[1], drone_pose_aux[2],
             atan2(-q_camera_target[1], -q_camera_target[0]));
  }
  return yaw;
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

bool backendSolver::checkConnectivity() {
  // check the connectivity with drones and target
  size_t cont = 0;

  for(auto it = uavs_pose_.begin();it!=uavs_pose_.end();it++){
    if(it->second.has_pose) cont++;
  }
  if(target_){
    if(target_has_pose) cont++;
    return (cont == drones.size()+1);
  }
  else
  {
    return (cont == drones.size());     
  }
}

void backendSolver::publishTrajectory() {

  optimal_control_interface::Solver traj;
  geometry_msgs::PoseStamped        pos;
  geometry_msgs::Twist              vel;

  for (int i = 0; i < time_horizon_; i++) {
    // trajectory to visualize
    pos.pose.position.x = solution_[i].pose.x;
    pos.pose.position.y = solution_[i].pose.y;
    pos.pose.position.z = solution_[i].pose.z;
    traj.positions.push_back(pos);
    vel.linear.x =solution_[i].velocity.x;
    vel.linear.y =solution_[i].velocity.y;
    vel.linear.z =solution_[i].velocity.z;
    traj.velocities.push_back(vel);
  }
  solved_trajectory_pub.publish(traj);
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
    point_distance = sqrt(pow((solution_[i].pose.x - uavs_pose_[drone_id_].state.pose.x), 2) + pow((solution_[i].pose.y - uavs_pose_[drone_id_].state.pose.y), 2) +
                          pow((solution_[i].pose.z - uavs_pose_[drone_id_].state.pose.z), 2));
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
  new_initial_guess = true; //TODO: for testing
  if (new_initial_guess) {
    std::array<float, 2> aux;
    // calculate scalar direction
    float aux_norm       = sqrt(pow((desired_odometry_.pose.pose.position.x - uavs_pose_[drone_id_].state.pose.x), 2) +
                          pow((desired_odometry_.pose.pose.position.y - uavs_pose_[drone_id_].state.pose.y), 2) +
                          pow((desired_odometry_.pose.pose.position.z - uavs_pose_[drone_id_].state.pose.z), 2));
    float scalar_dir_x   = (desired_odometry_.pose.pose.position.x - uavs_pose_[drone_id_].state.pose.x) / aux_norm;
    float scalar_dir_y   = (desired_odometry_.pose.pose.position.y - uavs_pose_[drone_id_].state.pose.y) / aux_norm;
    float scalar_dir_z   = (desired_odometry_.pose.pose.position.z - uavs_pose_[drone_id_].state.pose.z) / aux_norm;
    float vel_module_cte = max_vel / 2;  // vel cte guess for the initial
    // accelerations
    initial_guess_[0].acc.x = ZERO;
    initial_guess_[0].acc.y = ZERO;
    initial_guess_[0].acc.z = ZERO;
    initial_guess_[0].pose.x = uavs_pose_[drone_id_].state.pose.x;
    initial_guess_[0].pose.y = uavs_pose_[drone_id_].state.pose.y;
    initial_guess_[0].pose.z = uavs_pose_[drone_id_].state.pose.z;
    initial_guess_[0].velocity.x = scalar_dir_x * vel_module_cte;
    initial_guess_[0].velocity.y = scalar_dir_y * vel_module_cte;
    initial_guess_[0].velocity.z = scalar_dir_z * vel_module_cte;
    for (int i = 1; i < time_horizon_; i++) {
      initial_guess_[i].acc.x = ZERO;
      initial_guess_[i].acc.y = ZERO;
      initial_guess_[i].acc.z = ZERO;
      initial_guess_[i].velocity.x = scalar_dir_x * vel_module_cte;
      initial_guess_[i].velocity.y = scalar_dir_y * vel_module_cte;
      initial_guess_[i].velocity.z = scalar_dir_z * vel_module_cte;
      initial_guess_[i].pose.x = initial_guess_[i - 1].pose.x + step_size * initial_guess_[i - 1].velocity.x;
      initial_guess_[i].pose.y = initial_guess_[i - 1].pose.y + step_size * initial_guess_[i - 1].velocity.y;
      initial_guess_[i].pose.z = initial_guess_[i - 1].pose.z + step_size * initial_guess_[i - 1].velocity.z;
      // no fly zone
      if (pow(initial_guess_[i].pose.x - no_fly_zone_center_[0], 2) + pow(initial_guess_[i].pose.y - no_fly_zone_center_[1], 2) < pow(NO_FLY_ZONE_RADIUS, 2)) {
        aux                     = expandPose(initial_guess_[i].pose.x, initial_guess_[i].pose.y);
        initial_guess_[i].pose.x = aux[0];
        initial_guess_[i].pose.y = aux[1];
      }
    }
  } else {
    // previous one
    for (int i = 0; i < time_horizon_; i++) {
      initial_guess_[i].acc.x =solution_[i].acc.x;
      initial_guess_[i].acc.y =solution_[i].acc.y;
      initial_guess_[i].acc.z =solution_[i].acc.z;
      initial_guess_[i].pose.x =solution_[i].pose.x;
      initial_guess_[i].pose.y =solution_[i].pose.y;
      initial_guess_[i].pose.z = solution_[i].pose.z;
      initial_guess_[i].velocity.x =solution_[i].velocity.x;
      initial_guess_[i].velocity.y =solution_[i].velocity.y;
      initial_guess_[i].velocity.z =solution_[i].velocity.z;
    }
  }
  // for (int i = 0; i < time_horizon_; i++) {
  //     initial_guess_["pitch"][i] = 0.3;
  // }
  // log the initial guess
  logger->logging();
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
  float rounded_time = round( diff.count() * 10.0 ) / 10.0;
  return rounded_time;
}


void backendSolver::stateMachine() {
  int       closest_point = 0;
  ros::Rate solver_timer(solver_rate_); //Hz
  bool      loop_rate_violated = false;
  std::chrono::system_clock::time_point start;
  std::chrono::duration<double> diff;
  float actual_cicle_time = 0.0;
  bool change_initial_guess = true;
  // int cont =
  first_time_solving_ = true;
  
  while (ros::ok) {
    ros::spinOnce();
    if (desired_type_ == shot_executer::DesiredShot::IDLE) { // IDLE STATE
      IDLEState();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    } else if (desired_type_ == shot_executer::DesiredShot::GOTO || desired_type_ == shot_executer::DesiredShot::SHOT) { // Shooting action
       

      do {
        ros::spinOnce();
        // predict the target trajectory if it exists
        if (target_) {  
          targetTrajectoryVelocityCTEModel();
        }
        // if it is the first time or the previous time the solver couldn't success, don't take previous trajectory as initial guess
        calculateInitialGuess(first_time_solving_ || change_initial_guess);
        
        // call the solver
        solver_success = solver_pt_->solverFunction(desired_odometry_, no_fly_zone_center_, target_trajectory_, uavs_pose_, actual_cicle_time, first_time_solving_);  // ACADO
        // solver_success = solver_.solverFunction(initial_guess_,ax_,ay_,az_,x_,y_,z_,vx_,vy_,vz_, desired_odometry_,
        // no_fly_zone_center_,target_trajectory_,uavs_pose_);   // call the solver function  FORCES_PRO.h
        
        // log solved trajectory
        logger->loggingCalculatedTrajectory(solver_success);

        // if the solver didn't success, change initial guess
        if(solver_success !=0){
          change_initial_guess = true;
        }else{
          change_initial_guess = false;
        }  
      } while (solver_success != returnValueType::SUCCESSFUL_RETURN && solver_success != returnValueType::RET_MAX_TIME_REACHED);
    }

    // wait for the planned time
    if(solver_timer.sleep()){
      actual_cicle_time = 1/solver_rate_;
    }else{
      actual_cicle_time = round(solver_timer.cycleTime().toSec() * 10.0 )/ 10.0;
    }
    // check and log the time that the last loop lasted
    // csv_pose << "cycle time: " << actual_cicle_time << std::endl;
    
    // publish the last calculated trajectory if the solver successed
    saveCalculatedTrajectory();
    // check if the trajectory last the planned time, if not discard the navigated points. First time does not discard points
    if(actual_cicle_time>1/solver_rate_ && !first_time_solving_){
      closest_point = (actual_cicle_time-1/solver_rate_)/step_size;
    }else{
      closest_point = 0;
    }
    // predict yaw and pitch and publish trajectory
    std::vector<double> yaw   = predictingYaw();
    std::vector<double> pitch = predictingPitch();
    publishSolvedTrajectory(yaw, pitch, closest_point);
    logger->publishPath(); // publish to visualize

    first_time_solving_=false;
  
    solver_timer.reset();
  }
}