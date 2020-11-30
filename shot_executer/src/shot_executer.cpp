#include <shot_executer.h>
#ifdef MULTIDRONE
    #include <multidrone_msgs/ExecuteAction.h>
    #include <multidrone_msgs/DroneAction.h>
#endif


ShotExecuter::ShotExecuter(ros::NodeHandle &_nh,ros::NodeHandle &_pnh){

    // publisher
    desired_pose_pub_ = _pnh.advertise<shot_executer::DesiredShot>("desired_pose",10);
    target_trajectory_pub_ = _pnh.advertise<nav_msgs::Path>("target_trajectory_prediction",10);
    desired_pose_publisher = _pnh.advertise<geometry_msgs::PointStamped>("desired_point", 1);

    // subscriber
    target_pose_sub_ = _pnh.subscribe("target_topic",10,&ShotExecuter::targetPoseCallback,this);
    // client
    // this service start a callback that keeps publishing the desired pose
    shooting_action_srv_ = _nh.advertiseService("action",&ShotExecuter::actionCallback,this);
}


/** \brief this function publish the desired pose in a ros point type
 *  \param x,y,z        Desired pose
 */
void ShotExecuter::publishDesiredPoint(nav_msgs::Odometry desired_odometry) {
  geometry_msgs::PointStamped desired_point;
  desired_point.point.x         = desired_odometry.pose.pose.position.x;
  desired_point.point.y         = desired_odometry.pose.pose.position.y;
  desired_point.point.z         = desired_odometry.pose.pose.position.z;
  desired_point.header.frame_id = "uav1/gps_origin";
  desired_point.header.stamp = ros::Time::now();

  desired_pose_publisher.publish(desired_point);
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
    // that is valid if the velocity is not equal to zero or threshold

    const float xy_module = sqrt(pow(_msg->twist.twist.linear.x,2)+pow(_msg->twist.twist.linear.y,2));
    if(xy_module>MIN_XY_VEL){
        tf::Quaternion q(   
            _msg->pose.pose.orientation.x,
            _msg->pose.pose.orientation.y,
            _msg->pose.pose.orientation.z,
            _msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(target_orientation_[ROLL], target_orientation_[PITCH], target_orientation_[YAW]);
    }
    bool xy_small = (xy_module<MIN_XY_VEL);
    std::cout<<"target pose received and xy small: "<<xy_small<<std::endl;
}

std::vector<nav_msgs::Odometry> ShotExecuter::targetTrajectoryPrediction(){
    std::vector<nav_msgs::Odometry> target_trajectory;
    nav_msgs::Odometry aux;
    nav_msgs::Path path_to_publish;  //rviz
    geometry_msgs::PoseStamped aux_path;

    // velocity constant model for target trajectory prediction
    for(int i=0; i<time_horizon_;i++){
        aux.pose.pose.position.x = target_pose_.pose.pose.position.x+step_size_*i*target_pose_.twist.twist.linear.x;
        aux.pose.pose.position.y = target_pose_.pose.pose.position.y+step_size_*i*target_pose_.twist.twist.linear.y;
        aux.pose.pose.position.z = target_pose_.pose.pose.position.z+step_size_*i*target_pose_.twist.twist.linear.z;
        aux.twist.twist = target_pose_.twist.twist;
        // to visualize
        aux_path.pose.position.x = aux.pose.pose.position.x;
        aux_path.pose.position.y = aux.pose.pose.position.y;
        aux_path.pose.position.z = aux.pose.pose.position.z;

        target_trajectory.push_back(aux);
        path_to_publish.poses.push_back(aux_path);
    }

    path_to_publish.header.frame_id ="uav"+std::to_string(drone_id_)+"/gps_origin";
    target_trajectory_pub_.publish(path_to_publish);
    return target_trajectory;
}



shot_executer::DesiredShot ShotExecuter::calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory){


    // publish the orientation of the desired point as the orientation of the camera
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, camera_angles_[PITCH], camera_angles_[YAW]);  // Create this quaternion from roll/pitch/YAW (in radian
    //int dur = (int)(shooting_duration*10);
    nav_msgs::Odometry desired_point;
    shot_executer::DesiredShot desired_shot;
    desired_point.pose.pose.orientation.x = myQuaternion.getX();
    desired_point.pose.pose.orientation.y = myQuaternion.getY();
    desired_point.pose.pose.orientation.z = myQuaternion.getZ();
    desired_point.pose.pose.orientation.w = myQuaternion.getW();

    switch(_shooting_action.shooting_action_type){
    case shot_executer::ShootingAction::Request::IDLE:
        desired_shot.type = shot_executer::DesiredShot::IDLE;
        return desired_shot;
    //TODO
    case shot_executer::ShootingAction::Request::GOTO:
        desired_point.pose.pose.position.x  = _shooting_action.rt_parameters.x;
        desired_point.pose.pose.position.y = _shooting_action.rt_parameters.y;
        desired_point.pose.pose.position.z = _shooting_action.rt_parameters.z;

        // desired vel
        desired_point.twist.twist.linear.x =0.0;
        desired_point.twist.twist.linear.y =0.0;
        desired_point.twist.twist.linear.z =0.0;
        desired_shot.desired_odometry = desired_point;
        desired_shot.type = shot_executer::DesiredShot::GOTO;
        return desired_shot;
    
    case shot_executer::ShootingAction::Request::ELEVATOR:
        desired_point.pose.pose.position.x  = drone_pose_.pose.pose.position.x;//target_trajectory.back().pose.pose.position.x+_shooting_action.rt_parameters.x; //-10 //+(cos(-0.9)*_shooting_action.rt_parameters.x-sin(-0.9)*_shooting_action.rt_parameters.y);
        desired_point.pose.pose.position.y = drone_pose_.pose.pose.position.y;
        desired_point.pose.pose.position.z = target_trajectory[time_horizon_-1].pose.pose.position.z+_shooting_action.rt_parameters.z;
        // desired vel
        desired_point.twist.twist.linear.x =0;
        desired_point.twist.twist.linear.y =0;
        desired_point.twist.twist.linear.z =target_trajectory.back().twist.twist.linear.z;
        desired_shot.desired_odometry = desired_point;
        desired_shot.type = shot_executer::DesiredShot::SHOT;
        return desired_shot;


    case shot_executer::ShootingAction::Request::FOLLOW:
        desired_point.pose.pose.position.x  = target_trajectory[time_horizon_-1].pose.pose.position.x+(cos(target_orientation_[YAW])*_shooting_action.rt_parameters.x-sin(target_orientation_[YAW])*_shooting_action.rt_parameters.y);//target_trajectory.back().pose.pose.position.x+_shooting_action.rt_parameters.x; //-10 //+(cos(-0.9)*_shooting_action.rt_parameters.x-sin(-0.9)*_shooting_action.rt_parameters.y);
        desired_point.pose.pose.position.y = target_trajectory[time_horizon_-1].pose.pose.position.y+(sin(target_orientation_[YAW])*_shooting_action.rt_parameters.x+cos(target_orientation_[YAW])*_shooting_action.rt_parameters.y);
        desired_point.pose.pose.position.z = target_trajectory[time_horizon_-1].pose.pose.position.z+_shooting_action.rt_parameters.z;
        // desired vel
        desired_point.twist.twist.linear.x =target_trajectory.back().twist.twist.linear.x;
        desired_point.twist.twist.linear.y =target_trajectory.back().twist.twist.linear.y;
        desired_point.twist.twist.linear.z =0;
        desired_shot.desired_odometry = desired_point;
        desired_shot.type = shot_executer::DesiredShot::SHOT;
        return desired_shot;

    case shot_executer::ShootingAction::Request::FLYOVER:
        desired_point.pose.pose.position.x  = target_trajectory[time_horizon_-1].pose.pose.position.x+(cos(target_orientation_[YAW])*_shooting_action.rt_parameters.x-sin(target_orientation_[YAW])*_shooting_action.rt_parameters.y);
        desired_point.pose.pose.position.y = target_trajectory[time_horizon_-1].pose.pose.position.y+(sin(target_orientation_[YAW])*_shooting_action.rt_parameters.x+cos(target_orientation_[YAW])*_shooting_action.rt_parameters.y);
        desired_point.pose.pose.position.z  = _shooting_action.rt_parameters.z;

        // desired
        desired_point.twist.twist.linear.x =target_trajectory[time_horizon_-1].twist.twist.linear.x;
        desired_point.twist.twist.linear.y =target_trajectory[time_horizon_-1].twist.twist.linear.y;
        desired_point.twist.twist.linear.z =0;
        desired_shot.desired_odometry = desired_point;
        desired_shot.type = shot_executer::DesiredShot::SHOT;
        return desired_shot;
    
    case shot_executer::ShootingAction::Request::ESTABLISH:
        desired_point.pose.pose.position.x  = _shooting_action.rt_parameters.x;
        desired_point.pose.pose.position.y =  _shooting_action.rt_parameters.y;
        desired_point.pose.pose.position.z  = _shooting_action.rt_parameters.z;

        // desired
        desired_point.twist.twist.linear.x =0;
        desired_point.twist.twist.linear.y =0;
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
        std::vector<nav_msgs::Odometry> target_trajectory = targetTrajectoryPrediction();
        shot_executer::DesiredShot desired_shot = calculateDesiredPoint(shooting_action,target_trajectory);
        publishDesiredPoint(desired_shot.desired_odometry);
        // publish desired pose
        desired_shot.desired_odometry.header.frame_id = "uav"+std::to_string(drone_id_)+"/gps_origin";
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


#ifdef UAL
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
#endif


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
