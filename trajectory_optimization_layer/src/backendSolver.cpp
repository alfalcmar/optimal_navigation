#include<backendSolver.h>
#include <iostream>

backendSolver::backendSolver(ros::NodeHandle pnh, ros::NodeHandle nh){
    ROS_INFO("backend solver constructor");
      // parameters
    pnh.param<float>("solver_rate", solver_rate_, 4); // solver rate
   
    if (ros::param::has("~drones")) {
        if(!ros::param::get("~drones",drones)){
            ROS_ERROR("'Drones' does not have the rigth type");
        }
    }
    else {
        ROS_ERROR("fail to get the drones ids");
    }
    if (ros::param::has("~drone_id")) {
        ros::param::get("~drone_id",drone_id_);
    }
    else {
        ROS_ERROR("fail to get the drones id");
    }
    // parameters
    for(int i=0; i<drones.size(); i++){
        has_poses[TARGET] = false;
        has_poses[drones[i]] = false;
    }

    calculateNoFlyZonePoints(no_fly_zone_center_[0],no_fly_zone_center_[1],NO_FLY_ZONE_RADIUS);
    // subscripions
    

    desired_pose_sub = nh.subscribe<shot_executer::DesiredShot>("desired_pose",1,&backendSolver::desiredPoseCallback,this); // desired pose from shot executer
    // publishers
    desired_pose_publisher = pnh.advertise<geometry_msgs::PointStamped>("desired_point",1);
    solved_trajectory_pub = pnh.advertise<optimal_control_interface::Solver>("trajectory",1);
    path_rviz_pub = pnh.advertise<nav_msgs::Path>("path",1);
    path_no_fly_zone = pnh.advertise<nav_msgs::Path>("noflyzone",1);   
    target_path_rviz_pub = pnh.advertise<nav_msgs::Path>("target/path",1);

    // initialize map
    // std::array<float, time_horizon_> aux_for_initialization;
    // initial_guess_["ax"] = aux_for_initialization;
    // initial_guess_["ay"] = aux_for_initialization;
    // initial_guess_["az"] = aux_for_initialization;
    // initial_guess_["px"] = aux_for_initialization;
    // initial_guess_["py"] = aux_for_initialization;
    // initial_guess_["pz"] = aux_for_initialization;
    // initial_guess_["vx"] = aux_for_initialization;
    // initial_guess_["vy"] = aux_for_initialization;
    // initial_guess_["vz"] = aux_for_initialization;
    acado_solver_pt_ = new ACADOsolver;
    main_thread_ = std::thread(&backendSolver::stateMachine,this);


    // TODO integrate it into the class UAL interface
    // pose and trajectory subscriptions
   /** for(int i=0; i<drones.size(); i++){ // for each drone, subscribe to the calculated trajectory and the drone pose
        drone_pose_sub[drones[i]] = pnh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drones[i])+"/ual/pose", 10, std::bind(&uavPoseCallback, std::placeholders::_1, drones[i]));               // Change for each drone ID
        if(drones[i] !=drone_id_){
            drone_trajectory_sub[drones[i]] = pnh.subscribe<optimal_control_interface::Solver>("/drone_"+std::to_string(drones[i])+"/solver", 1, std::bind(&uavTrajectoryCallback, std::placeholders::_1, drones[i]));
        }
        //initialize
        trajectory_solved_received[drones[i]] = false;     
    }*/


    // log files
    csv_pose.open("/home/alfonso/trajectories"+std::to_string(drone_id_)+".csv");
    //csv_record.open("/home/alfonso/to_reproduce"+std::to_string(drone_id_)+".csv");
    csv_pose << std::fixed << std::setprecision(5);
    //csv_record << std::fixed << std::setprecision(5);
    //csv_debug.open("/home/alfonso/debug_"+std::to_string(drone_id_)+".csv");
}


/** \brief Callback for the target pose
 */
void backendSolverUAL::targetPoseCallbackGRVC(const nav_msgs::Odometry::ConstPtr &msg)
{   
    has_poses[TARGET] = true;
    target_odometry_.pose.pose = msg->pose.pose;
}

void backendSolver::desiredPoseCallback(const shot_executer::DesiredShot::ConstPtr &msg)
{   

    desired_odometry_.pose = msg->desired_odometry.pose;
    desired_type_ = msg->type;
    //ROS_INFO("Desired pose received: x: %f y: %f z: %f",msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z]);
}

/** \brief uav odometry callback (mrs system)
 */
void backendSolverMRS::uavCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    uavs_pose_[drone_id_] = *msg;
    has_poses[drone_id_] = true; 
}



/** \brief This callback receives the solved trajectory of uavs
 */
void backendSolver::uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id){
    uavs_trajectory[id].positions.clear();
    uavs_trajectory[id].velocities.clear();
    uavs_trajectory[id].accelerations.clear();
    trajectory_solved_received[id] = true;
    uavs_trajectory[id].positions = msg->positions;
    uavs_trajectory[id].velocities = msg->velocities;
    uavs_trajectory[id].accelerations = msg->accelerations;
    ROS_INFO("Solver %d: trajectory callback from drone %d",drone_id_,id);
}

/** \brief callback for the pose of uavs
 */
void backendSolver::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id){
    has_poses[id] = true;
    if(!trajectory_solved_received[id]){
        for(int i=0; i<time_horizon_;i++){
            geometry_msgs::PoseStamped pose_aux;
            pose_aux.pose.position.x = msg->pose.position.x;
            pose_aux.pose.position.y = msg->pose.position.y;
            pose_aux.pose.position.z = msg->pose.position.z;
            uavs_trajectory[id].positions.push_back(pose_aux);
        }
    }
    uavs_pose_[id].pose.pose = msg->pose; 
}


void backendSolverMRS::targetCallbackMRS(const nav_msgs::Odometry::ConstPtr& _msg) 
{
    has_poses[TARGET] = true;
    target_odometry_ = *_msg;
}

void backendSolver::publishSolvedTrajectory(const std::vector<double> &yaw,const std::vector<double> &pitch, const int delayed_points /*0 default */){
    ROS_INFO("virtual definition of publish solved trajectory");
}

void backendSolver::publishTrajectory(){
    
    optimal_control_interface::Solver traj;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    for(int i=0;i<time_horizon_; i++){
        //trajectory to visualize
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


bool backendSolver::desiredPoseReached(const std::vector<double> desired_pos, const std::vector<double> last_traj_pos){
    Eigen::Vector3f desired_pose = Eigen::Vector3f(desired_pos[0],desired_pos[1],desired_pos[2]);
    Eigen::Vector3f final_pose = Eigen::Vector3f(last_traj_pos[0],last_traj_pos[1],last_traj_pos[2]);
    if((desired_pose-final_pose).norm()<REACHING_TOLERANCE){
        return true;
    }else{
        return false;
    }
}
std::vector<double> backendSolver::predictingPitch(){
    
    std::vector<double> pitch;
    
    Eigen::Vector3f target_pose_aux;
    Eigen::Vector3f drone_pose_aux;
    Eigen::Vector3f q_camera_target;
    for(int i=0;i<time_horizon_;i++){
        target_pose_aux = Eigen::Vector3f(target_trajectory_[i].pose.pose.position.x,target_trajectory_[i].pose.pose.position.y,0);
        drone_pose_aux = Eigen::Vector3f(x_[i],y_[i],z_[i]);
        q_camera_target = drone_pose_aux-target_pose_aux;
        float aux_sqrt = sqrt(pow(q_camera_target[0], 2.0)+pow(q_camera_target[1],2.0));
        pitch.push_back(1.57- atan2(aux_sqrt,q_camera_target[2]));
    }  
    return pitch;
}

/** utility function to move yaw pointing the target*/
std::vector<double> backendSolver::predictingYaw(){
    
    std::vector<double> yaw;
    
    Eigen::Vector3f target_pose_aux;
    Eigen::Vector3f drone_pose_aux;
    Eigen::Vector3f q_camera_target;
    for(int i=0;i<time_horizon_;i++){
        target_pose_aux = Eigen::Vector3f(target_trajectory_[i].pose.pose.position.x,target_trajectory_[i].pose.pose.position.y,target_trajectory_[i].pose.pose.position.z);
        drone_pose_aux = Eigen::Vector3f(x_[i],y_[i],z_[i]);
        q_camera_target = drone_pose_aux-target_pose_aux;
        yaw.push_back(atan2(-q_camera_target[1],-q_camera_target[0]));
    }
    return yaw;
}
void backendSolver::pruebaDroneSubida(){
    
    double aux = 0.0;
    double vel = 0.5;
    for(int i=0; i<time_horizon_;i++){
        double aux = uavs_pose_[drone_id_].pose.pose.position.z + step_size*i*vel;
        z_[i]=aux;
    }
}

void backendSolver::targetTrajectoryVelocityCTEModel(){
    
    target_trajectory_.clear();
    double target_vel_module = sqrt(pow(target_odometry_.twist.twist.linear.x,2)+pow(target_odometry_.twist.twist.linear.y,2));
    nav_msgs::Odometry aux;
    for(int i=0; i<time_horizon_;i++){
        aux.pose.pose.position.x = target_odometry_.pose.pose.position.x + step_size*i*target_odometry_.twist.twist.linear.x;
        aux.pose.pose.position.y = target_odometry_.pose.pose.position.y + step_size*i*target_odometry_.twist.twist.linear.y;
        aux.twist = target_odometry_.twist; // velocity constant model
        target_trajectory_.push_back(aux);
    }
}

geometry_msgs::Quaternion backendSolver::toQuaternion(const double pitch, const double roll, const double yaw)
{
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
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(time_horizon_);
    msg.header.frame_id = "uav1/gps_origin";
    for (int i = 0; i < time_horizon_; i++) {
        poses.at(i).pose.position.x = x_[i];
        poses.at(i).pose.position.y = y_[i];
        poses.at(i).pose.position.z = z_[i];
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
void backendSolver::publishDesiredPoint(){
    geometry_msgs::PointStamped desired_point;
    desired_point.point.x = desired_odometry_.pose.pose.position.x;
    desired_point.point.y = desired_odometry_.pose.pose.position.y;
    desired_point.point.z = desired_odometry_.pose.pose.position.z;
    desired_point.header.frame_id = "uav1/gps_origin";

    desired_pose_publisher.publish(desired_point);
}

/** \brief Construct the no fly zone approximated by a tetrahedron to visualize it on RVIZ 
 *  \param  2D points
*/
void backendSolver::publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]){
    nav_msgs::Path msg;
    msg.header.frame_id = "uav1/gps_origin";

    std::vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;

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

void backendSolver::logToCSVCalculatedTrajectory(int solver_success){
    csv_pose<<"solver_success: "<<solver_success<<std::endl;
    csv_pose<<"Calculated trajectroy"<<std::endl;
     for(int i=0; i<time_horizon_; i++){
        csv_pose <<ax_[i] << ", " << ay_[i] << ", " << az_[i]<< ", "<<x_[i] << ", " << y_[i] << ", " << z_[i]<< ", "<< vx_[i]<< ", " <<vy_[i]<< ", " <<vz_[i]<<std::endl;
    }
}

void backendSolver::logToCsv(){
    // logging all results
    csv_pose<<"Time horizon"<<time_horizon_<<std::endl;
    csv_pose<<"My pose: "<<uavs_pose_[drone_id_].pose.pose.position.x<<", "<<uavs_pose_[drone_id_].pose.pose.position.y<<", "<<uavs_pose_[drone_id_].pose.pose.position.z<<std::endl;
    csv_pose<<"My vel: "<<uavs_pose_[drone_id_].twist.twist.linear.x<<", "<<uavs_pose_[drone_id_].twist.twist.linear.y<<", "<<uavs_pose_[drone_id_].twist.twist.linear.z<<std::endl;
    csv_pose<<"My accel: "<<0.0<<", "<<0.0<<", "<<0.0<<std::endl;
    //logging inter-uavs pose
    if(multi_){
        csv_pose<<"Drone 2: "<<uavs_pose_[2].pose.pose.position.x <<", "<< uavs_pose_[2].pose.pose.position.y<< ", "<<uavs_pose_[2].pose.pose.position.z<<", "<< std::endl;
        csv_pose<<"Drone 3: "<<uavs_pose_[3].pose.pose.position.x <<", "<< uavs_pose_[3].pose.pose.position.y<<", "<< uavs_pose_[3].pose.pose.position.z<<", "<< std::endl;
    }
    csv_pose<<"initial guess: "<<std::endl;
    for(int i=0; i<time_horizon_;i++){
        csv_pose <<initial_guess_["ax"][i] << ", " << initial_guess_["ay"][i] << ", " << initial_guess_["az"][i]<< ", "<<
                    initial_guess_["px"][i] << ", " << initial_guess_["py"][i] << ", " << initial_guess_["pz"][i]<<", "<<
                    initial_guess_["vx"][i] << ", " << initial_guess_["vy"][i] << ", " << initial_guess_["vz"][i]<<std::endl;
    }
    // csv_pose<<"Calculated trajectroy"<<std::endl;
    //  for(int i=0; i<x_.size(); i++){
    //     csv_pose <<ax_[i] << ", " << ay_[i] << ", " << az_[i]<< ", "<<x_[i] << ", " << y_[i] << ", " << z_[i]<< ", "<< vx_[i]<< ", " <<vy_[i]<< ", " <<vz_[i]<<std::endl;
    // }
}

nav_msgs::Path backendSolver::targetPathVisualization()
{
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(target_trajectory_.size());
    msg.header.frame_id = "uav1/gps_origin";
    for (int i = 0; i < target_trajectory_.size(); i++) {
        poses.at(i).pose.position.x = target_trajectory_[i].pose.pose.position.x;
        poses.at(i).pose.position.y = target_trajectory_[i].pose.pose.position.y;
        poses.at(i).pose.position.z = target_trajectory_[i].pose.pose.position.z;
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    return msg;
}

bool backendSolver::checkConnectivity(){
    // check the connectivity with drones and target
    int cont = 0;   
    for(int i=0; i<drones.size(); i++){ // check drones pose
        if(has_poses[drones[i]] == true){
            cont++;
        }
    }
    if(target_){ // if target included 
        if(has_poses[TARGET]){ // check target pose
        cont++;
        }
        if(cont==drones.size()+1){ //+1 because the target
            return true;
        }else{
            return false;
        }
    }else{
        if(cont==drones.size()){
            return true;
        }else{
            return false;
        }
    } 
}

void backendSolver::calculateNoFlyZonePoints(const float x_center, const float y_center, const float radius){
    no_fly_zone_points_.clear();
    std::array<float,2> aux;
    const float SAFETY_OFFSET = 0.1;
    for(float angle =0.0; angle<2*M_PI;angle = angle+0.25){
        aux[0] = x_center+(radius+SAFETY_OFFSET)*cos(angle);
        aux[1] = y_center+(radius+SAFETY_OFFSET)*sin(angle);
        no_fly_zone_points_.push_back(aux);
    }
}

int backendSolver::closestPose(){
    float nearest_distance = sqrt(pow((x_[0]-uavs_pose_[drone_id_].pose.pose.position.x),2)+
                                pow((y_[0]-uavs_pose_[drone_id_].pose.pose.position.y),2)+
                                pow((z_[0]-uavs_pose_[drone_id_].pose.pose.position.z),2));;
    float point_distance = INFINITY;
    for(int i = 1; i<time_horizon_;i++){
        point_distance = sqrt(  pow((x_[i]-uavs_pose_[drone_id_].pose.pose.position.x),2)+
                                pow((y_[i]-uavs_pose_[drone_id_].pose.pose.position.y),2)+
                                pow((z_[i]-uavs_pose_[drone_id_].pose.pose.position.z),2)); 
        if(point_distance<nearest_distance){
            nearest_distance = point_distance;
        }
        else
        {
            return i;
        }
    }
}

bool backendSolver::isDesiredPoseReached(const nav_msgs::Odometry &_desired_pose, const nav_msgs::Odometry &_last_pose){
    ROS_INFO("");
}

void backendSolver::calculateInitialGuess(){
    ROS_INFO("intial guess");
    
    if (first_time_solving_)
    {
            // calculate scalar direction
        float aux_norm = sqrt(  pow((desired_odometry_.pose.pose.position.x-uavs_pose_[drone_id_].pose.pose.position.x),2)+
                                pow((desired_odometry_.pose.pose.position.y-uavs_pose_[drone_id_].pose.pose.position.y),2)+
                                pow((desired_odometry_.pose.pose.position.z-uavs_pose_[drone_id_].pose.pose.position.z),2)); 
        float scalar_dir_x = (desired_odometry_.pose.pose.position.x-uavs_pose_[drone_id_].pose.pose.position.x)/aux_norm;
        float scalar_dir_y = (desired_odometry_.pose.pose.position.y-uavs_pose_[drone_id_].pose.pose.position.y)/aux_norm;
        float scalar_dir_z = (desired_odometry_.pose.pose.position.z-uavs_pose_[drone_id_].pose.pose.position.z)/aux_norm;
        float vel_module_cte = max_vel/2; // vel cte guess for the initial 
        // accelerations
        initial_guess_["ax"][0] = ZERO;
        initial_guess_["ay"][0] = ZERO;
        initial_guess_["az"][0] = ZERO;
        initial_guess_["px"][0] = uavs_pose_[drone_id_].pose.pose.position.x;
        initial_guess_["py"][0] = uavs_pose_[drone_id_].pose.pose.position.y;
        initial_guess_["pz"][0] = uavs_pose_[drone_id_].pose.pose.position.z;
        initial_guess_["vx"][0] = scalar_dir_x*vel_module_cte;
        initial_guess_["vy"][0] = scalar_dir_y*vel_module_cte;
        initial_guess_["vz"][0] = scalar_dir_z*vel_module_cte;
        for (int i = 1; i<time_horizon_;i++){
            initial_guess_["ax"][i] = ZERO;
            initial_guess_["ay"][i] = ZERO;
            initial_guess_["az"][i] = ZERO;
            initial_guess_["vx"][i] = scalar_dir_x*vel_module_cte;
            initial_guess_["vy"][i] = scalar_dir_y*vel_module_cte;
            initial_guess_["vz"][i] = scalar_dir_z*vel_module_cte;
            initial_guess_["px"][i] = initial_guess_["px"][i-1]+step_size* initial_guess_["vx"][i-1];
            initial_guess_["py"][i] = initial_guess_["py"][i-1]+step_size* initial_guess_["vy"][i-1];
            initial_guess_["pz"][i] = initial_guess_["pz"][i-1]+step_size* initial_guess_["vz"][i-1];
        }
    
    }else{
    
        //previous one
        for(int i = 0;i<time_horizon_;i++){
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
    // sequrity check

    // saturate velocity
    float aux_val;
    std::array<float,2> aux;

    for(int i=0; i<time_horizon_;i++){
        initial_guess_["vx"][i] = std::min(std::max( (float)initial_guess_["vx"][i], -max_vel), max_vel); // saturating velocity

        initial_guess_["vy"][i] = std::min(std::max((float) initial_guess_["vy"][i], -max_vel), max_vel);

        initial_guess_["vz"][i] =  std::min(std::max( (float)initial_guess_["vz"][i], -max_vel), max_vel);
        // no fly zone
        //  if(sqrt(pow(initial_guess_["px"][i]-no_fly_zone_center_[0],2)+pow(initial_guess_["py"][i]-no_fly_zone_center_[1],2))<pow(NO_FLY_ZONE_RADIUS,2)){
        //     aux = expandPose(initial_guess_["px"][i],initial_guess_["py"][i]);
        //     initial_guess_["px"][i] = aux[0];
        //     initial_guess_["px"][i] = aux[1];
        // }
    }
}

std::array<float,2> backendSolver::expandPose(float x, float y){
   float aux_norm = 0.0;
   float minor_distance = INFINITY;
   std::array<float,2> point_to_return;

   for(int i=0;i<no_fly_zone_points_.size();i++){
        aux_norm = sqrt(  pow((no_fly_zone_points_[i][0]-x),2)+
                          pow((no_fly_zone_points_[i][1]-y),2));
        if(aux_norm<minor_distance){
            minor_distance = aux_norm;
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
    //setInitialPose();
    first_activation_ = false;
  }
  if (activated_ == req.data) { //if it was already in the state that is received
    res.success = false;
    if (req.data) { // activated
      res.message = "Planning is already activated.";
      ROS_ERROR("%s", res.message.c_str());
    } else {    // deactivated
      res.message = "Planning is already deactivated.";
      ROS_ERROR("%s", res.message.c_str());
    }
    return true;
  } 
  if (req.data) { 
    if (!planning_done_) {
      res.message = "Planning activated.";
      res.success = true;
      ROS_WARN("%s", res.message.c_str());
      activated_ = req.data;
    } else {
      res.message = "Planning cannot be activated because it is finished. Call reset service.";
      res.success = false;
      ROS_ERROR("%s", res.message.c_str());
    }
  } else {
    subida = true;
    res.message = "Planning deactivated.";
    res.success = true;
    ROS_WARN("%s", res.message.c_str());
    activated_ = req.data;
  }
}

void backendSolver::staticLoop(){
    solver_rate_ = solver_rate_static_;
    for(int i=0; i<time_horizon_;i++){ // maintain the position
        x_[i]=uavs_pose_[drone_id_].pose.pose.position.x;
        y_[i]=uavs_pose_[drone_id_].pose.pose.position.y;
    }
    // TODO think about what to do with that
    if(subida){
        pruebaDroneSubida();
    }else{
        for(int i=0;i<time_horizon_;i++){
            z_[i]=uavs_pose_[drone_id_].pose.pose.position.z;
        }
    }
   
    targetTrajectoryVelocityCTEModel();
    std::vector<double> yaw = predictingYaw();
    std::vector<double> pitch = predictingPitch();
 

    publishSolvedTrajectory(yaw,pitch);
}

void backendSolver::IDLEState(){
    ROS_INFO("Solver %d: IDLE state",drone_id_);
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
int backendSolver::checkDelay(std::chrono::system_clock::time_point start){
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - start;
    int number_of_points = diff.count()/step_size;
    return number_of_points;
}
void backendSolver::stateMachine(){
    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> diff;
    while(ros::ok){
        
        if(desired_type_ == shot_executer::DesiredShot::IDLE){
            IDLEState();
        }
        else if( desired_type_ == shot_executer::DesiredShot::GOTO || desired_type_ == shot_executer::DesiredShot::SHOT){
            solver_rate_ = solver_rate_dynamic_;
            if(target_){  // calculate the target trajectory if it exists
                targetTrajectoryVelocityCTEModel();
            }
            ros::spinOnce();
            start = std::chrono::system_clock::now();
            calculateInitialGuess();
            first_time_solving_ = false;
            logToCsv();

            if(desired_type_ == shot_executer::DesiredShot::GOTO){
                 solver_success = acado_solver_pt_->solverFunction(initial_guess_, ax_,ay_,az_,x_,y_,z_,vx_,vy_,vz_,desired_odometry_, obst_,target_trajectory_,uavs_pose_); // ACADO
            }
            else if(desired_type_ == shot_executer::DesiredShot::SHOT){
                 solver_success = acado_solver_pt_->solverFunction2D(initial_guess_, ax_,ay_,az_,x_,y_,z_,vx_,vy_,vz_,desired_odometry_, obst_,target_trajectory_,uavs_pose_); // ACADO

                //solver_success = solver_.solverFunction(initial_guess_,ax_,ay_,az_,x_,y_,z_,vx_,vy_,vz_, desired_odometry_, obst_,target_trajectory_,uavs_pose_);   // call the solver function  FORCES_PRO.h     
            }
            if(solver_success==0){
                std::vector<double> yaw = predictingYaw();
                std::vector<double> pitch = predictingPitch();
                //TODO where is going pitch and yaw?
                int closest_point = closestPose();
                //deletingPoints(delayed_points);
                csv_pose<<"delayed_points: "<<closest_point<<std::endl;
                publishSolvedTrajectory(yaw,pitch,closest_point);
            }else{
                first_time_solving_ = true;
            }
            logToCSVCalculatedTrajectory(solver_success);
            diff = std::chrono::system_clock::now()-start;
            csv_pose << "delay: " <<diff.count() << " s\n";
            //publishDesiredPoint();
            publishPath();
        }

        ros::Rate r(1/solver_rate_);
        sleep(solver_rate_);
        ros::spinOnce();
    }

        
}

backendSolverMRS::backendSolverMRS(ros::NodeHandle &_pnh, ros::NodeHandle &_nh) : backendSolver::backendSolver(_pnh,_nh){
    ROS_INFO("Leader constructor");
    std::string target_topic;
    _pnh.param<std::string>("target_topic",target_topic, "/gazebo/dynamic_target/jeff_electrician/pose"); // target topic   /gazebo/dynamic_target/dynamic_pickup/pose
    target_array_sub = _pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, &backendSolverMRS::targetCallbackMRS,this); //target pose
    mrs_trajectory_tracker_pub = _nh.advertise<mrs_msgs::TrackerTrajectory>("control_manager/mpc_tracker/set_trajectory",1);
    solved_trajectory_MRS_pub = _nh.advertise<formation_church_planning::Trajectory>("formation_church_planning/planned_trajectory",1);
    uav_odometry_sub = _nh.subscribe<nav_msgs::Odometry>("odometry/odom_main",1, &backendSolverMRS::uavCallback,this);
    diagnostics_pub = _nh.advertise<formation_church_planning::Diagnostic>("formation_church_planning/diagnostics",1);
    service_for_activation = _nh.advertiseService("formation_church_planning/toggle_state", &backendSolverMRS::activationServiceCallback,this);

    ros::Rate rate(1); //hz
    while(!checkConnectivity()){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Solver %d is not ready",drone_id_);
    }
    is_initialized = true;
    diagnostic_timer_ = _nh.createTimer(ros::Duration(0.5), &backendSolverMRS::diagTimer,this);
    //main_thread_ = std::thread(&backendSolverMRS::stateMachine,this);

    ROS_INFO("Solver %d is ready", drone_id_);
}


void backendSolverMRS::publishSolvedTrajectory(const std::vector<double> &yaw,const std::vector<double> &pitch, const int closest_point){
    
    mrs_msgs::TrackerPoint aux_point;
    formation_church_planning::Point aux_point_for_followers;
    mrs_msgs::TrackerTrajectory traj_to_command;
    formation_church_planning::Trajectory traj_to_followers;
    traj_to_command.fly_now = true;
    traj_to_command.use_yaw = true;    
    // check that _x _y _z are the same size
    for(int i=closest_point;i<time_horizon_; i++){
    
        //trajectory to command
        aux_point.x = x_[i];
        aux_point.y = y_[i];
        aux_point.z = z_[i];
        aux_point.yaw = yaw[i];
        //trajectory to followers
        aux_point_for_followers.x = x_[i];
        aux_point_for_followers.y = y_[i];
        aux_point_for_followers.z = z_[i];
        aux_point_for_followers.yaw = yaw[i];
        aux_point_for_followers.pitch = pitch[i];
        aux_point_for_followers.phi = 0.0;
        aux_point_for_followers.mode = 2;

        traj_to_followers.points.push_back(aux_point_for_followers);
        traj_to_command.points.push_back(aux_point);
    }
    solved_trajectory_MRS_pub.publish(traj_to_followers);   
    mrs_trajectory_tracker_pub.publish(traj_to_command);
}

void backendSolverMRS::diagTimer(const ros::TimerEvent &event) {
  if (!is_initialized){
    return;
  }
  formation_church_planning::Diagnostic diag_msg;
  diag_msg.header.stamp               = ros::Time::now();
  diag_msg.uav_name                   = "uav"+std::to_string(drone_id_);
  diag_msg.robot_role                 = "leader";
  diag_msg.state                      = "waiting_in_initial_position";
  diag_msg.flying_mode                = "1";
  diag_msg.ready                      = true;
  diag_msg.dist_from_desired_position = 0;
  diag_msg.last_time_solution_found = ros::Time::now() - ros::Time::now(); // alternatively: ros::Duration(0, 0);
  diag_msg.planning_activated = true;
  diag_msg.planning_finished  = true;
  try {
    ROS_INFO_ONCE("[%s]: Publishing diag message.", ros::this_node::getName().c_str());
    diagnostics_pub.publish(diag_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", diagnostics_pub.getTopic().c_str());
  }
}


void backendSolverUAL::ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg){
    ual_state_.state = msg->state;
}

void backendSolverUAL::ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    uavs_pose_[drone_id_].twist.twist = msg->twist;
}

backendSolverUAL::backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh) : backendSolver::backendSolver(_pnh,_nh){
    sub_velocity = _nh.subscribe<geometry_msgs::TwistStamped>("ual/velocity",1,&backendSolverUAL::ownVelocityCallback,this);
    uav_state_sub = _nh.subscribe<uav_abstraction_layer::State>("ual/state",1,&backendSolverUAL::ualStateCallback,this);
      // TODO include or not calls to the uav from this node
    //go_to_waypoint_client = pnh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    // take_off_srv = pnh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    // ros::Subscriber target_pose_sub = pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallbackGRVC);
    //set_velocity_pub = pnh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
    // main loop
}