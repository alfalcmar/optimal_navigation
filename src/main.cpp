#include "FORCESNLPsolver.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
//#include "uav_path_manager/GeneratePath.h"
//#include "uav_path_manager/GetGeneratedPath.h"
#include <thread>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <optimal_control_interface.h>
#include <uav_abstraction_layer/State.h>
#include <multidrone_msgs/DroneAction.h>
#include <multidrone_msgs/ExecuteAction.h>
#include <actionlib/server/simple_action_server.h>
#include <multidrone_msgs/TargetStateArray.h>


ros::Publisher set_pose_pub;
ros::Publisher set_velocity_pub;
ros::Subscriber uav_state_sub;
actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>* server_;
ros::ServiceClient go_to_waypoint_client;
ros::ServiceClient take_off_srv;

// solver output
std::vector<double> x;   
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;


std::vector<double> desired_wp{0,0,0}; 
std::vector<double> desired_vel{0,0,0};
std::vector<double> obst;
std::vector<double> target_vel = {0, 0};

// solver variables
int solver_success;
float solver_rate;

// ual variables
int ual_state;

/** target array for real experiment
 */
void targetarrayCallback(const multidrone_msgs::TargetStateArray::ConstPtr& _msg) // real target callback
{
    
    has_poses[0] = true;

  //gimbal target
    for (auto target:_msg->targets) {
        target_pose.pose = target.pose.pose;
    }
}

/** \brief This callback receives the solved trajectory of uavs
 */
void uavTrajectoryCallback(const multidrone_msgs::SolvedTrajectory::ConstPtr &msg, int id){
    uavs_trajectory[id].clear();
    trajectory_solved_received[id] = true;
    uavs_trajectory[id] = msg->positions;
    ROS_INFO("Solver %d: trajectory callback from drone %d",drone_id,id);
}
/** \brief Preempt function for ros actions
 */
void preemptCallback(){

}

/** \brief function to calculate if the trajectory calculated by the solver finishes in the desired pose
 */
bool desiredPoseReached(const double x_des, const double y_des, const double z_des, const double x_traj, const double y_traj, const double z_traj){
    Eigen::Vector3f desired_pose = Eigen::Vector3f(x_des,y_des,z_des);
    Eigen::Vector3f final_pose = Eigen::Vector3f(x_traj,y_traj,z_traj);
    if((desired_pose-final_pose).norm()<2.0){
        return true;
    }else{
        return false;
    }

}

/** \brief utility function to publish marker for the desired pose
 */

/** \brief thread for multidrone shooting action
 */

void shootingActionThread(){
    shooting_action_running = true;
    ROS_INFO("Executer %d: Shooting action thread initilized",drone_id);
    bool duration_reached = false;
    ros::Rate rate(solver_rate); //hz
    bool desired_point_reached = false;
    /* main loop to call the solver. */
    while(ros::ok && !duration_reached && !desired_point_reached){
        ros::spinOnce();
        // solver function
          x.clear();
          y.clear();
          z.clear();
          vx.clear();
          vy.clear();
          vz.clear();
        solver_success = solverFunction(x,y,z,vx,vy,vz, desired_wp, desired_vel, obst,target_vel);
        if(solver_success==1){
            if(shooting_action_type == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL){
                desired_point_reached = desiredPoseReached(f_pose[0],f_pose[1], f_pose[2],desired_wp[0],desired_wp[1],desired_wp[2]);
            }else if(shooting_action_type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY){
                desired_point_reached = desiredPoseReached(f_pose[0],f_pose[1], f_pose[2],x[time_horizon-1],y[time_horizon-1],z[time_horizon-1]);
            }
            publishTrajectory(x,y,z,vx,vy,vz);

            // log solver output to csv file
            logToCsv(x,y,z,vx,vy,vz);
            // publish path to rviz visualizer

            // double point_1[2]= {-13.1,-35.55};
            // double point_2[2]= {-2.2,-20.8};
            // double point_3[2]= {10.77,-39.7};
            // double point_4[2]= {-2.5,-51.3};

            // publishNoFlyZone(point_1,point_2,point_3,point_4);

        }
        
        publishDesiredPoint(desired_wp[0], desired_wp[1], desired_wp[2]);

        if(drone_id==1){

            nav_msgs::Path msg;
            std::vector<geometry_msgs::PoseStamped> poses(target_trajectory.size());
            msg.header.frame_id = "map";
            for (int i = 0; i < target_trajectory.size(); i++) {
                poses.at(i).pose.position.x = target_trajectory[i].x;
                poses.at(i).pose.position.y = target_trajectory[i].y;
                poses.at(i).pose.position.z = target_trajectory[i].z;
                poses.at(i).pose.orientation.x = 0;
                poses.at(i).pose.orientation.y = 0;
                poses.at(i).pose.orientation.z = 0;
                poses.at(i).pose.orientation.w = 1;
            }
            msg.poses = poses;
            target_path_rviz_pub.publish(msg);
        
        }
        ros::spinOnce();
        rate.sleep();
    }

ROS_INFO("Executer %d: Finishing shooting actin thread",drone_id);

}
/** \brief callback for multidrone action client
 */

void actionCallback(){

    const multidrone_msgs::DroneAction goal =server_->acceptNewGoal()->action_goal;
    
    //duration =  goal.shooting_action.duration;

    if(goal.action_type == multidrone_msgs::DroneAction::TYPE_SHOOTING){
        target_final_pose[0] = goal.shooting_action.rt_trajectory[goal.shooting_action.rt_trajectory.size()-1].point.x;
        target_final_pose[1] = goal.shooting_action.rt_trajectory[goal.shooting_action.rt_trajectory.size()-1].point.y;

        try{
            shooting_action_type = goal.shooting_action.shooting_roles.at(0).shooting_type.type;
        }
        catch(std::out_of_range o){
            ROS_ERROR("trying to shooting roles");
        }
        for(int i = 0; i<goal.shooting_action.shooting_roles[0].shooting_parameters.size(); i++){
            std::cout<<goal.shooting_action.shooting_roles[0].shooting_parameters[i].param<<std::endl;
           shooting_parameters[goal.shooting_action.shooting_roles[0].shooting_parameters[i].param] = goal.shooting_action.shooting_roles[0].shooting_parameters[i].value;
        }

        if(shooting_action_running) stop_current_shooting = true;   // If still validating, end the current validation to start the new one as soon as possible.
        if(shooting_action_thread.joinable()) shooting_action_thread.join();
        shooting_action_thread = std::thread(shootingActionThread);
        return;
    }
    /*else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_TAKEOFF){ // TAKE OFF NAVIGATION ACTION
        // taking off
        uav_abstraction_layer::TakeOff srv;
        srv.request.blocking = true;
        srv.request.height = goal.path[0].point.z;
        if(!take_off_srv.call(srv)){
            ROS_WARN("Drone %d: the take off is not available",drone_id);
            
        }else{
            ROS_INFO("Drone %d: taking_off",drone_id);
            multidrone_msgs::ExecuteResult result;
            result.goal_achieved = true;
            server_->setSucceeded(result);
        }
    }else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_LAND){ // LAND NAVIGATION ACTION

    }else if(goal.action_type == multidrone_msgs::DroneAction::TYPE_GOTOWAYPOINT){ // GO TO WAYPOINT NAVIGATION ACTION
        uav_abstraction_layer::GoToWaypoint go_to_waypoint_srv;
        geometry_msgs::PoseStamped setpoint_pose;
        multidrone_msgs::ExecuteResult result;
        int i=0;
        for(i; i<goal.path.size();i++){
            setpoint_pose.header.stamp     = ros::Time::now();
            setpoint_pose.header.frame_id  = "map";
            setpoint_pose.pose.position.x  = goal.path[i].point.x;
            setpoint_pose.pose.position.y  = goal.path[i].point.y;
            setpoint_pose.pose.position.z  = goal.path[i].point.z;
            setpoint_pose.pose.orientation = uavs_pose[drone_id].pose.orientation;  // the same orientation as the previous waypoint
            go_to_waypoint_srv.request.waypoint = setpoint_pose;
            go_to_waypoint_srv.request.blocking = true;
            if(go_to_waypoint_client.call(go_to_waypoint_srv)){
                ROS_INFO("Executer %d: calling the go_to_waypoint service",drone_id);
            }else
            {
                ROS_WARN("Executer %d: go_to_waypoint service is not available",drone_id);
            } 
        }

        if (goal.final_yaw_if_gotowaypoint.x==0.0 && goal.final_yaw_if_gotowaypoint.y==0.0 && goal.final_yaw_if_gotowaypoint.z==0.0 && goal.final_yaw_if_gotowaypoint.w==0.0) {
            ROS_INFO("finish go to waypoint");
            result.goal_achieved=true;
            server_->setSucceeded(result);
            return;
        }
        ROS_INFO("girando en yaw");
        setpoint_pose.pose.position.x  = goal.path[goal.path.size()-1].point.x;
        setpoint_pose.pose.position.y  = goal.path[goal.path.size()-1].point.y;
        setpoint_pose.pose.position.z  = goal.path[goal.path.size()-1].point.z;
        setpoint_pose.pose.orientation.x  = goal.final_yaw_if_gotowaypoint.x;
        setpoint_pose.pose.orientation.y  = goal.final_yaw_if_gotowaypoint.y;
        setpoint_pose.pose.orientation.z  = goal.final_yaw_if_gotowaypoint.z;
        setpoint_pose.pose.orientation.w = goal.final_yaw_if_gotowaypoint.w;        
        go_to_waypoint_srv.request.waypoint = setpoint_pose;
        go_to_waypoint_srv.request.blocking = true;
        result.goal_achieved = go_to_waypoint_client.call(go_to_waypoint_srv);
        if(result.goal_achieved){
            ROS_INFO("Executer %d: calling the go_to_waypoint service",drone_id);
        }else
        {
            ROS_WARN("Executer %d: go_to_waypoint service is not available",drone_id);
        }
        server_->setSucceeded(result);

    }
    */
}


/** \brief callback for the pose of uavs
 */

void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id){
    has_poses[id] = true;
    if(!trajectory_solved_received[id]){
        for(int i=0; i<time_horizon;i++){
            geometry_msgs::Point pose_aux;
            pose_aux.x = msg->pose.position.x;
            pose_aux.y = msg->pose.position.y;
            pose_aux.z = msg->pose.position.z;
            uavs_trajectory[id].push_back(pose_aux);
        }
    }
    uavs_pose[id].pose = msg->pose;
    
}

/** callback for ual state
 */

void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg){
    ual_state = msg->state;
}

/** Callback for the target pose
 */

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    if(has_poses[0] == false){
        target_init.pose = msg->pose.pose;
    }
    has_poses[0] = true;
    target_pose.pose = msg->pose.pose;
}


/** main thread
 */

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "optimal_control_interface_node");

    //utility vars
    int n_steps;
    ros::NodeHandle nh = ros::NodeHandle();

    // parameters
    nh.param<float>("solver_rate", solver_rate, 0.5);
    std::string target_topic;
    nh.param<std::string>("target_topic",target_topic, "/drc_vehicle_xp900/odometry");
 
    // parameters
    if (ros::param::has("~drones")) {
        ros::param::get("~drones",drones);
    }
    else {
        ROS_WARN("fail to get the drones ids");
    }
    if (ros::param::has("~drone_id")) {
        ros::param::get("~drone_id",drone_id);
    }
    else {
        ROS_WARN("fail to get the drones id");
    }
    if (ros::param::has("~target_final_pose")) {
        ros::param::get("~target_final_pose",target_final_pose);
    }
    else {
        ROS_WARN("fail to get final target pose");
    }
    if (ros::param::has("~desired_wp")) {
        ros::param::get("~desired_wp",desired_wp);
    }
    else {
        ROS_WARN("fail to get desidred wp");
    }

    if (ros::param::has("~desired_vel")) {
        ros::param::get("~desired_vel",desired_vel);
    }
    else {
        ROS_WARN("fail to get desired_vel");
    }
    if (ros::param::has("~no_fly_zone")) {
        ros::param::get("~no_fly_zone",obst);
    }
    else {
        ROS_WARN("fail to get no_fly_zone");
    }
    //action service
    server_ = new actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>(nh, "/drone_"+std::to_string(drone_id)+"/action_server_smooth", false);
    server_->registerGoalCallback(boost::bind(&actionCallback));
    server_->registerPreemptCallback(boost::bind(&preemptCallback));
    server_->start(); 


    // subscribers and publishers
    uav_state_sub = nh.subscribe<uav_abstraction_layer::State>("ual/state",1,ualStateCallback);
    go_to_waypoint_client = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
   // ros::Subscriber target_pose_sub = nh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
    ros::Subscriber target_array_sub = nh.subscribe<multidrone_msgs::TargetStateArray>("/target_3d_state", 1, targetarrayCallback);

    set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ual/set_pose",1);
    set_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);

    
    for(int i=0; i<drones.size(); i++){
        drone_pose_sub[drones[i]] = nh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drones[i])+"/ual/pose", 10, std::bind(&uavPoseCallback, std::placeholders::_1, drones[i]));               // Change for each drone ID
        if(drones[i] !=drone_id){
            drone_trajectory_sub[drones[i]] = nh.subscribe<multidrone_msgs::SolvedTrajectory>("/drone_"+std::to_string(drones[i])+"/solver", 1, std::bind(&uavTrajectoryCallback, std::placeholders::_1, drones[i]));
        }
        //initialize
        trajectory_solved_received[drones[i]] = false;     
    }
    // init solver
    init(nh);



    ros::spin();
    
    
    //threadObjSolver.join();
    return 0;
}
