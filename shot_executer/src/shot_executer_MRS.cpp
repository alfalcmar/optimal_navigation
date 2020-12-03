#include<shot_executer_MRS.h>
ShotExecuterMRS::ShotExecuterMRS(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) : ShotExecuter::ShotExecuter(_nh,_pnh, "uav1/gps_origin"){
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

    uav_odometry_sub = _nh.subscribe<nav_msgs::Odometry>("odometry/odom_main",1, &ShotExecuterMRS::uavCallback, this);

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