#include <shot_executer.h>
#ifdef MULTIDRONE
    #include <multidrone_msgs/ExecuteAction.h>
    #include <multidrone_msgs/DroneAction.h>
#endif


ShotExecuter::ShotExecuter(ros::NodeHandle &_nh,ros::NodeHandle &_pnh){
    //params
    std::string target_topic = "";
    _nh.param<std::string>("target_topic",target_topic, "/gazebo/dynamic_target/jeff_electrician/pose");
    std::string prediction_mode = "";   
    _pnh.param<std::string>("prediction_mode",prediction_mode, "orientation"); // add pnh to the constructor
    if(prediction_mode == "velocity"){
        prediction_mode_ = VELOCITY_MODE;
    }else if(prediction_mode == "orientation"){
        prediction_mode_ = ORIENTATION_MODE;
    }else{
        ROS_WARN("Shot executer %d: invalid prediction mode. Orientation mode set by default");
    }
    // publisher
    desired_pose_pub_ = _nh.advertise<shot_executer::DesiredShot>("desired_pose",10);
    target_trajectory_pub_ = nh.advertise<nav_msgs::Path>("target_trajectory_prediction",10);
    // subscriber
    target_pose_sub_ = _nh.subscribe(target_topic,10,&ShotExecuter::targetPoseCallback,this);
    // client

    // service
    shooting_action_srv_ = _nh.advertiseService("action",&ShotExecuter::actionCallback,this);
    

}

/** \brief 
 */
void ShotExecuter::publishCameraCommand(){
    ROS_WARN("Publish camera command of the base class");
}
/** \brief target array for real experiment
 */
void ShotExecuter::targetPoseCallback(const nav_msgs::Odometry::ConstPtr& _msg) // real target callback
{
    target_pose_ = *_msg;
}


std::vector<nav_msgs::Odometry> ShotExecuter::targetTrajectoryPrediction(){
    //double target_vel_module = sqrt(pow(target_vel[0],2)+pow(target_vel[1],2));
    std::vector<nav_msgs::Odometry> target_trajectory;
    nav_msgs::Odometry aux;
    nav_msgs::Path path_to_publish;  //rviz
    geometry_msgs::PoseStamped aux_path;

    if(prediction_mode_ == VELOCITY_MODE){
        for(int i=0; i<time_horizon_;i++){
            aux.pose.pose.position.x = target_pose_.pose.pose.position.x+ step_size_*i*target_pose_.twist.twist.linear.x;
            aux.pose.pose.position.y = target_pose_.pose.pose.position.y + step_size_*i*target_pose_.twist.twist.linear.y;
            aux.pose.pose.position.z = target_pose_.pose.pose.position.z + step_size_*i*target_pose_.twist.twist.linear.z;
            aux_path.pose.position.x = aux.pose.pose.position.x;
            aux_path.pose.position.y = aux.pose.pose.position.y;
            aux_path.pose.position.z = aux.pose.pose.position.z;
            target_trajectory.push_back(aux);
            path_to_publish.poses.push_back(aux_path);
        }
    }
    else if(prediction_mode_ == ORIENTATION_MODE){
        double roll,pitch,yaw;
         tf::Quaternion q(   
            target_pose_.pose.pose.orientation.x,
            target_pose_.pose.pose.orientation.y,
            target_pose_.pose.pose.orientation.z,
            target_pose_.pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        double norm = sqrt(pow(target_pose_.pose.pose.position.x,2)+pow(target_pose_.pose.pose.position.y,2));
        double velx = VEL_CTE*cos(yaw);
        double vely = VEL_CTE*sin(yaw);
        double velz = 0.0;
        for(int i=0; i<time_horizon_;i++){
            aux.pose.pose.position.x = target_pose_.pose.pose.position.x+step_size_*i*velx;
            aux.pose.pose.position.y = target_pose_.pose.pose.position.y+step_size_*i*vely;
            aux.pose.pose.position.z = target_pose_.pose.pose.position.z+step_size_*i*velz;
            aux_path.pose.position.x = aux.pose.pose.position.x;
            aux_path.pose.position.y = aux.pose.pose.position.y;
            aux_path.pose.position.z = aux.pose.pose.position.z;
            target_trajectory.push_back(aux);
            path_to_publish.poses.push_back(aux_path);
        }
    }else{
        ROS_ERROR("invalid mode");
    }
    path_to_publish.header.frame_id ="uav1/gps_origin";
    target_trajectory_pub_.publish(path_to_publish);
    return target_trajectory;
}


shot_executer::DesiredShot ShotExecuter::calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory){

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, camera_angles_[PITCH], camera_angles_[YAW]);  // Create this quaternion from roll/pitch/YAW (in radian
    //int dur = (int)(shooting_duration*10);
    nav_msgs::Odometry desired_point;
    shot_executer::DesiredShot desired_shot;
    desired_point.pose.pose.orientation.x = myQuaternion.getX();
    desired_point.pose.pose.orientation.y = myQuaternion.getY();
    desired_point.pose.pose.orientation.z = myQuaternion.getZ();
    desired_point.pose.pose.orientation.w = myQuaternion.getW();

    double roll,pitch,yaw;
    tf::Quaternion q(   
        target_pose_.pose.pose.orientation.x,
        target_pose_.pose.pose.orientation.y,
        target_pose_.pose.pose.orientation.z,
        target_pose_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

        switch(_shooting_action.shooting_action_type){
        //TODO
        case shot_executer::ShootingAction::Request::GOTO:
            desired_point.pose.pose.position.x  = _shooting_action.rt_parameters.x;
            desired_point.pose.pose.position.y = _shooting_action.rt_parameters.y;
            desired_point.pose.pose.position.z = drone_pose_.pose.pose.position.z;

            // desired vel
            desired_point.twist.twist.linear.x =0.0;
            desired_point.twist.twist.linear.y =0.0;
            desired_point.twist.twist.linear.z =0.0;
            desired_shot.desired_odometry = desired_point;
            desired_shot.type = shot_executer::DesiredShot::GOTO;
            return desired_shot;

        case shot_executer::ShootingAction::Request::FOLLOW:
            desired_point.pose.pose.position.x  = _shooting_action.rt_parameters.x;//target_trajectory.back().pose.pose.position.x+_shooting_action.rt_parameters.x; //-10 //+(cos(-0.9)*_shooting_action.rt_parameters.x-sin(-0.9)*_shooting_action.rt_parameters.y);
            desired_point.pose.pose.position.y = _shooting_action.rt_parameters.y;//target_trajectory.back().pose.pose.position.y+_shooting_action.rt_parameters.y;//+(sin(-0.9)*_shooting_action.rt_parameters.x+cos(-0.9)*_shooting_action.rt_parameters.y);
            desired_point.pose.pose.position.z = drone_pose_.pose.pose.position.z;

            // desired vel
            desired_point.twist.twist.linear.x =target_trajectory.back().twist.twist.linear.x;
            desired_point.twist.twist.linear.y =target_trajectory.back().twist.twist.linear.y;
            desired_point.twist.twist.linear.z =0;
            desired_shot.desired_odometry = desired_point;
            desired_shot.type = shot_executer::DesiredShot::SHOT;
            return desired_shot;

        case shot_executer::ShootingAction::Request::FLYOVER:
            desired_point.pose.pose.position.x  = target_trajectory[time_horizon_-1].pose.pose.position.x+(cos(yaw)*_shooting_action.rt_parameters.x-sin(yaw)*_shooting_action.rt_parameters.y);
            desired_point.pose.pose.position.y = target_trajectory[time_horizon_-1].pose.pose.position.y+(sin(yaw)*_shooting_action.rt_parameters.x+cos(yaw)*_shooting_action.rt_parameters.y);
            desired_point.pose.pose.position.z  = drone_pose_.pose.pose.position.z;

            // desired
            desired_point.twist.twist.linear.x =target_trajectory[time_horizon_-1].twist.twist.linear.x;
            desired_point.twist.twist.linear.y =target_trajectory[time_horizon_-1].twist.twist.linear.y;
            desired_point.twist.twist.linear.z =0;
            desired_shot.desired_odometry = desired_point;
            desired_shot.type = shot_executer::DesiredShot::SHOT;
            return desired_shot;

        default:
            ROS_ERROR("Shooting action type invalid");
            return desired_shot;

    }
}

bool ShotExecuter::actionCallback(shot_executer::ShootingAction::Request  &req, shot_executer::ShootingAction::Response &res){
    ROS_INFO("action callback");
    struct shooting_action shooting_action;
    shooting_action.shooting_action_type = req.shooting_action_type;
    shooting_action.rt_parameters = req.rt_parameter;
    shooting_action.duration = req.duration;
    shooting_action.length = req.length;
    shooting_action.start_event = req.start_event;
    shooting_action.target_type = req.target_type;
    if(shooting_action_running_) 
    {
        new_shooting_action_received_ = true;
    } 
    if(action_thread_.joinable()){
        action_thread_.join();
    }
    action_thread_ = std::thread(&ShotExecuter::actionThread,this,shooting_action);
    new_shooting_action_received_ = false;
    res.received = true;
    return true;
}

void ShotExecuter::cameraThread(){
    ROS_INFO("camera thread initialized");
    ros::Rate rate(rate_camera_publisher_);
    while(ros::ok){
        publishCameraCommand();
        rate.sleep();
    }
}
/** \brief Shooting action thread callback
 */
void ShotExecuter::actionThread(struct shooting_action shooting_action){
    ROS_INFO("action thread");
    bool time_reached = false;
    bool distance_reached = false;
    ros::Rate rate(rate_pose_publisher_);  
    shooting_action_running_ = true;    
    while(!time_reached && !distance_reached && ros::ok() && !new_shooting_action_received_){
        //TODO predict
        std::vector<nav_msgs::Odometry> target_trajectory = targetTrajectoryPrediction();
        shot_executer::DesiredShot desired_shot = calculateDesiredPoint(shooting_action,target_trajectory);
        // publish desired pose
        desired_shot.desired_odometry.header.frame_id = "uav1/gps_origin";
        desired_pose_pub_.publish(desired_shot);
        ROS_INFO("desired_pose published");
        rate.sleep();
    }
    shooting_action_running_ = false;
}

void ShotExecuter::calculateGimbalAngles(){
    Eigen::Vector3f target_pose = Eigen::Vector3f(target_pose_.pose.pose.position.x,target_pose_.pose.pose.position.y,target_pose_.pose.pose.position.z);
    Eigen::Vector3f drone_pose = Eigen::Vector3f(drone_pose_.pose.pose.position.x,drone_pose_.pose.pose.position.y,drone_pose_.pose.pose.position.z);
    Eigen::Vector3f q_camera_target = drone_pose-target_pose;
    float aux_sqrt = sqrt(pow(q_camera_target[0], 2.0)+pow(q_camera_target[1],2.0));
    camera_angles_[PITCH] =1.57- atan2(aux_sqrt,q_camera_target[2]);  //-
    camera_angles_[YAW] = atan2(-q_camera_target[1],-q_camera_target[0]);
    //std::cout<<"YAW: "<<camera_angles_[YAW]<<std::endl;
    //std::cout<<"pitch: "<<camera_angles_[pitch]<<std::endl;

}

ShotExecuterMRS::ShotExecuterMRS(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) : ShotExecuter::ShotExecuter(_nh,_pnh){
     /* initialize service for arming */
    std::string service_name;
    service_name = "/uav"+std::to_string(drone_id_)+"/mavros/cmd/arming";
    arming_client_ = _nh.serviceClient<mavros_msgs::CommandBool>(service_name);
    /* initialize service for offboard */
    service_name = "/uav" + std::to_string(drone_id_) + "/mavros/set_mode";
    offboard_client_ = _nh.serviceClient<mavros_msgs::SetMode>(service_name);
    /* initialize service for motors */
    service_name = "/uav" + std::to_string(drone_id_) + "/control_manager/motors";
    motors_client_ = _nh.serviceClient<std_srvs::SetBool>(service_name);
    /* initialize service for takeoff */
    service_name = "/uav" + std::to_string(drone_id_) + "/uav_manager/takeoff";
    takeoff_client_ = _nh.serviceClient<std_srvs::Trigger>(service_name);

    std::string topic_name = "/uav" + std::to_string(drone_id_) + "/servo_camera/set_pitch";
    camera_pub_ = _nh.advertise<std_msgs::Float32>(topic_name,10);


    uav_odometry_sub = nh.subscribe<nav_msgs::Odometry>("odometry/odom_main",1, &ShotExecuterMRS::uavCallback, this);

    //callTakeOff();

    camera_thread_ = std::thread(&ShotExecuterMRS::cameraThread,this);
}

void ShotExecuterMRS::uavCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    drone_pose_ = *msg;
}

void ShotExecuterMRS::publishCameraCommand(){
    std_msgs::Float32 msg;
    calculateGimbalAngles();
    msg.data = camera_angles_[PITCH];
    camera_pub_.publish(msg);
}


bool ShotExecuterMRS::callTakeOff(){
  bool success = true;
  // motors on
  if (!all_motors_on_){
    sleep(1);
    std_srvs::SetBool srv;
    srv.request.data = true;
    srv.request.data = true;
    ROS_WARN("[%s]: Calling motors on.", ros::this_node::getName().c_str());
    
    motors_client_.call(srv);
    if (srv.response.success) {
    ROS_INFO("[%s]: %d motors on successfully called.", ros::this_node::getName().c_str(), drone_id_);
    } else {
    ROS_INFO("[%s]: %d motors on failed.", ros::this_node::getName().c_str(), drone_id_);
    success = false;
    }
    all_motors_on_ = success; 
  }
  //  arming
  if (!robot_armed_ && success) {
    mavros_msgs::CommandBool arming_srv;
    arming_srv.request.value = 1;
    ROS_WARN("[%s]: Starting arming of UAVs.", ros::this_node::getName().c_str());
    arming_client_.call(arming_srv);
    if (arming_srv.response.success) {
        ROS_INFO("[%s]: Arming of drone %d successfully activated.", ros::this_node::getName().c_str(), drone_id_);
    } else {
        ROS_WARN("[%s]: Arming of drone %d was not activated.", ros::this_node::getName().c_str(), drone_id_);
        success = false;
    } 
    robot_armed_ = success;
    ROS_INFO("[%s]: Arming called successfully.", ros::this_node::getName().c_str());
  }
  // offboard
  if (!robot_in_offboard_mode_ && success) {
    sleep(1);
    mavros_msgs::SetMode srv_set_mode;
    srv_set_mode.request.base_mode   = 0;
    srv_set_mode.request.custom_mode = "offboard";
    ROS_WARN("[%s]: Switching to offboard mode.", ros::this_node::getName().c_str());

    offboard_client_.call(srv_set_mode);
    if (srv_set_mode.response.mode_sent) {
    ROS_INFO("[%s]: drone %d was successfully switched to offboard mode.", ros::this_node::getName().c_str(), drone_id_);
    } else {
    ROS_INFO("[%s]: drone %d switched to offboard mode failed.", ros::this_node::getName().c_str(), drone_id_);
    success = false;
    }

    robot_in_offboard_mode_ = success;
    ROS_INFO_COND(robot_in_offboard_mode_, "[%s]: Switching to offboard mode was successful.", ros::this_node::getName().c_str());
  }
  // takeoff
  if (success) {
    sleep(1);
    ROS_WARN("[%s]: Calling takeoff.", ros::this_node::getName().c_str());
    std_srvs::Trigger srv_trigger;
    takeoff_client_.call(srv_trigger);
    if (srv_trigger.response.success) {
    ROS_INFO("[%s]: Takeoff of drone %d successfully activated.", ros::this_node::getName().c_str(), drone_id_);
    } else {
    ROS_WARN("[%s]: Takeoff of drone %d was not activated.", ros::this_node::getName().c_str(), drone_id_);
    success = false;
    }
  }
  takeoff_called_succesfully_ = takeoff_called_succesfully_ || success;
  ROS_INFO_COND(takeoff_called_succesfully_, "[%s]: ", ros::this_node::getName().c_str());
  return success;
}

ShotExecuterUAL::ShotExecuterUAL(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) : ShotExecuter::ShotExecuter(_nh,_pnh){

    go_to_waypoint_client_ = _nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    take_off_srv_ = _nh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    land_client_ = _nh.serviceClient<uav_abstraction_layer::Land>("ual/take_off");
}


bool ShotExecuterUAL::takeOff(const double _height){
    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = _height;
    if(!take_off_srv_.call(srv)){
        return false;
    }else{
        return true;
    }
}
#ifdef MULTIDRONE
ShotExecuterMultidrone::ShotExecuterMultidrone(ros::NodeHandle &_nh) : ShotExecuter(_nh){
    server_ = new actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>(_nh, "action_server", false);
    server_->registerGoalCallback(boost::bind(&ShotExecuterMultidrone::actionCallback, this));
    // server_->registerPreemptCallback(boost::bind(&Executer::preemptCallback, this));
    server_->start();
}
#endif

#ifdef MULTIDRONE
/** \brief go to waypoint off the drone
 *  \param _height   take off's height
 *  \return success
 **/
bool ShotExecuterMultidrone::goToWaypoint(const multidrone_msgs::DroneAction &_goal){
    uav_abstraction_layer::GoToWaypoint go_to_waypoint_srv;
    geometry_msgs::PoseStamped setpoint_pose;
    multidrone_msgs::ExecuteResult result;
    int i=0;
    for(i; i<_goal.path.size();i++){
        setpoint_pose.header.stamp     = ros::Time::now();
        setpoint_pose.header.frame_id  = "map";
        setpoint_pose.pose.position.x  = _goal.path[i].point.x;
        setpoint_pose.pose.position.y  = _goal.path[i].point.y;
        setpoint_pose.pose.position.z  = _goal.path[i].point.z;
        setpoint_pose.pose.orientation = drone_pose_.pose.pose.orientation;  // the same orientation as the previous waypoint
        go_to_waypoint_srv.request.waypoint = setpoint_pose;
        go_to_waypoint_srv.request.blocking = true;
        if(go_to_waypoint_client_.call(go_to_waypoint_srv)){
            ROS_INFO("Executer %d: calling the go_to_waypoint service",drone_id_);
        }else
        {
            ROS_WARN("Executer %d: go_to_waypoint service is not available",drone_id_);
        } 
    }
    if (_goal.final_yaw_if_gotowaypoint.x==0.0 && _goal.final_yaw_if_gotowaypoint.y==0.0 && _goal.final_yaw_if_gotowaypoint.z==0.0 && _goal.final_yaw_if_gotowaypoint.w==0.0) {
        ROS_INFO("finish go to waypoint");
        result.goal_achieved=true;
        server_->setSucceeded(result);
        return true;
    }
    ROS_INFO("girando en yaw");
    setpoint_pose.pose.position.x  = _goal.path[_goal.path.size()-1].point.x;
    setpoint_pose.pose.position.y  = _goal.path[_goal.path.size()-1].point.y;
    setpoint_pose.pose.position.z  = _goal.path[_goal.path.size()-1].point.z;
    setpoint_pose.pose.orientation.x  = _goal.final_yaw_if_gotowaypoint.x;
    setpoint_pose.pose.orientation.y  = _goal.final_yaw_if_gotowaypoint.y;
    setpoint_pose.pose.orientation.z  = _goal.final_yaw_if_gotowaypoint.z;
    setpoint_pose.pose.orientation.w = _goal.final_yaw_if_gotowaypoint.w;        
    go_to_waypoint_srv.request.waypoint = setpoint_pose;
    go_to_waypoint_srv.request.blocking = true;
    result.goal_achieved = go_to_waypoint_client_.call(go_to_waypoint_srv);
    if(result.goal_achieved){
        ROS_INFO("Executer %d: calling the go_to_waypoint service",drone_id_);
    }else
    {
        ROS_WARN("Executer %d: go_to_waypoint service is not available",drone_id_);
    }
    server_->setSucceeded(result);
}
#endif

#ifdef MULTIDRONE
/** \brief Shooting action thread callback
 */
void ShotExecuterMultidrone::actionThread(const multidrone_msgs::DroneAction goal){
    //duration =  goal.shooting_action.duration;
    if(goal.action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING){
        //TODO predict
        std::vector<nav_msgs::Odometry> target_trajectory = targetTrajectoryPrediction();
        // calculate pose
        std::map<std::string,float> shooting_parameters;
        /*try{
            shooting_action_type = goal.shooting_action.shooting_roles.at(0).shooting_type.type;
        }
        catch(std::out_of_range o){
            ROS_ERROR("trying to shooting roles");
        }*/
        for(int i = 0; i<goal.shooting_action.shooting_roles[0].shooting_parameters.size(); i++){
            std::cout<<goal.shooting_action.shooting_roles[0].shooting_parameters[i].param<<std::endl;
            shooting_parameters[goal.shooting_action.shooting_roles[0].shooting_parameters[i].param] = goal.shooting_action.shooting_roles[0].shooting_parameters[i].value;
        }
        nav_msgs::Odometry desired_pose = calculateDesiredPoint(goal.shooting_action.shooting_roles.at(0).shooting_type.type,shooting_parameters ,target_trajectory);
        // publish desired pose


        /** if(shooting_action_running) stop_current_shooting = true;   // If still validating, end the current validation to start the new one as soon as possible.
        if(shooting_action_thread.joinable()) shooting_action_thread.join();
        shooting_action_thread = std::thread(shootingActionThread);
        return;*/
    }
    else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF){ // TAKE OFF NAVIGATION ACTION
        if(takeOff(goal.path[0].point.z)){
            ROS_INFO("Drone %d: taking_off",drone_id_);
            multidrone_msgs::ExecuteResult result;
            result.goal_achieved = true;
            server_->setSucceeded(result);
        }else{
            ROS_WARN("Drone %d: the take off is not available",drone_id_);
        }
    }else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_LAND){ // LAND NAVIGATION ACTION

    }else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT){ // GO TO WAYPOINT NAVIGATION ACTION
        goToWaypoint(goal);
    }
}
#endif

#ifdef MULTIDRONE
/** \brief action callback
 *  \TODO reduce this function
 */
void ShotExecuterMultidrone::actionCallback(){
    const multidrone_msgs::DroneAction goal =server_->acceptNewGoal()->action_goal;
    action_thread_ = std::thread(&ShotExecuterMultidrone::actionThread, this, goal);
    
}

/** \brief Calculate desired pose. If type flyby, calculate wrt last mission pose .If lateral, calculate wrt time horizon pose
 *  \TODO   z position and velocity, angle relative to target
 *  \TODO   calculate orientation by velocity
 **/
nav_msgs::Odometry ShotExecuterMultidrone::calculateDesiredPoint(const int shooting_type, std::map<std::string, float> shooting_parameters, const std::vector<nav_msgs::Odometry> &target_trajectory){
    //int dur = (int)(shooting_duration*10);
    nav_msgs::Odometry desired_point;
    switch(shooting_type){
        //TODO
        case multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY:
            desired_point.pose.pose.position.x  = target_trajectory.back().pose.pose.position.x+(cos(-0.9)*shooting_parameters["x_e"]-sin(-0.9)*shooting_parameters["y_0"]);
            desired_point.pose.pose.position.y = target_trajectory.back().pose.pose.position.y+(sin(-0.9)*shooting_parameters["x_e"]+cos(-0.9)*shooting_parameters["y_0"]);
            desired_point.pose.pose.position.z = drone_pose_.pose.pose.position.z;

            // desired vel
            desired_point.twist.twist.linear.x =target_trajectory.back().twist.twist.linear.x;
            desired_point.twist.twist.linear.y =target_trajectory.back().twist.twist.linear.y;
            desired_point.twist.twist.linear.z =0;
        break;
        case multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL:
            desired_point.pose.pose.position.x  = target_trajectory[time_horizon-1].pose.pose.position.x-sin(-0.9)*shooting_parameters["y_0"];
            desired_point.pose.pose.position.y = target_trajectory[time_horizon-1].pose.pose.position.y+cos(-0.9)*shooting_parameters["y_0"];
            desired_point.pose.pose.position.z  = drone_pose_.pose.pose.position.z;

            // desired
            desired_point.twist.twist.linear.x =target_trajectory[time_horizon-1].twist.twist.linear.x;
            desired_point.twist.twist.linear.y =target_trajectory[time_horizon-1].twist.twist.linear.y;
            desired_point.twist.twist.linear.z =0;
        break;
    }
}
#endif
