#include "FORCESNLPsolver.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
//#include "uav_path_manager/GeneratePath.h"
//#include "uav_path_manager/GetGeneratedPath.h"
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <optimal_control_interface.h>
#include <multidrone_msgs/TargetStateArray.h>

// TODO implement interface between shot executer and this node (Desired pose)
// TODO think about who is going to take off, land, etc


/** this node is a c++ wrap to use optimization solvers with ROS for UAVs, 
 * In this case, this use the optimization_forces_pro_library, a library created to use this framework
 * to interface with the rest of the nodes.
 * In summary this node contains the following:
 * Callback for interfacing with the rest of the nodes:
 *     Callback for other's calculated trajectories
 *     Callback for Drones's pose
 *     Callback for Target Pose
 *     TODO Callback for the desired pose (In this case is received from the shot executer)
 *
 * 
 * functions for visualization and logging the output of this solver
 *   
 * 
 * Receive parameters that the user can change:
 *      Target topic
 *      Drone ids
 *      Own dron id
 * 
 * Initialize the optimal control interface node
 * Each solver frequency:
 *      clear the trajectories
 *      call the solver
 *      if the solver's call is successfully, send the trajectories to the trajectory follower
 * 
 * 
 * 
 * The variables to communicate with the solver library are:
 *     uavs_trajectory[id, trajectory]
 *     uavs_poses[id, pose]
 *     target_pose
 *     target_trajectory[]
 * **/

///////////////////// Callbacks //////////////////////////

/**
 */
void desiredPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{   
    desired_wp[0] = msg->pose.position.x;
    desired_wp[1] = msg->pose.position.y;
    desired_wp[2] = msg->pose.position.z;
    ROS_INFO("Desired pose received: x: %f y: %f z: %f",desired_wp[0],desired_wp[1],desired_wp[2]);
}
/** Drone velocity callback
 */
void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    own_velocity.twist = msg->twist;
}

/** \brief This callback receives the solved trajectory of uavs
 */
void uavTrajectoryCallback(const multidrone_msgs::SolvedTrajectory::ConstPtr &msg, int id){
    uavs_trajectory[id].clear();
    trajectory_solved_received[id] = true;
    uavs_trajectory[id] = msg->positions;
    ROS_INFO("Solver %d: trajectory callback from drone %d",drone_id,id);
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

//////////////////////// Visualization function //////////////////////////

/**
 */
nav_msgs::Path targetPathVisualization()
{
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
    return msg;
}

// utility vars

int solver_success;


// TODO use or remove
/** callback for ual state
 */

void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg){
    ual_state.state = msg->state;
}


 //////////////////// utility function ///////////////////////////////////

/** \brief Function to check connectivity between nodes
 * check /ual/pose
 * TODO check target pose
 */
bool checkConnectivity(){
    // check the connectivity with drones and target
    bool connectivity_done = false;
    int cont = 0;   
    for(int i=0; i<drones.size(); i++){ // check drones pose
        if(has_poses[drones[i]] == true){
            cont++;
        }
    }
    /*if(has_poses[0]){ // check target pose
        cont++;
    }*/
    if(cont==drones.size()){ //+1 if target
        connectivity_done = true;
    }
}


/** \brief Function to initialize the solver
 *  Check if the drone is subscribed to the others drone poses and target**/
bool init(ros::NodeHandle pnh){

    // parameters
    pnh.param<float>("solver_rate", solver_rate, 0.5);
    std::string target_topic;
    pnh.param<std::string>("target_topic",target_topic, "/drc_vehicle_xp900/odometry");
    if (ros::param::has("~drones")) {
        if(!ros::param::get("~drones",drones)){
            ROS_WARN("'Drones' does not have the rigth type");
            return false;
        }
    }
    else {
        ROS_WARN("fail to get the drones ids");
        return false;
    }
    if (ros::param::has("~drone_id")) {
        ros::param::get("~drone_id",drone_id);
    }
    else {
        ROS_WARN("fail to get the drones id");
    }
    // parameters
    if (ros::param::has("~priority")) {
        ros::param::get("~priority",priority);
    }
    else{
        ROS_WARN("Solver %d: fail to get the priority param",drone_id);
    }  // parameters

    for(int i=0; i<drones.size(); i++){
        has_poses[0] = false;
        has_poses[drones[i]] = false;
    }
    
    // subscripions
    sub_velocity = pnh.subscribe<geometry_msgs::TwistStamped>("ual/velocity",1,ownVelocityCallback);
    uav_state_sub = pnh.subscribe<uav_abstraction_layer::State>("ual/state",1,ualStateCallback); //ual state
    target_array_sub = pnh.subscribe<multidrone_msgs::TargetStateArray>("/target_3d_state", 1, targetarrayCallback); //target pose
    desired_pose_sub = pnh.subscribe<geometry_msgs::PoseStamped>("desired_pose",1,desiredPoseCallback);
    // publishers
    desired_pose_publisher = pnh.advertise<geometry_msgs::PointStamped>("solver/desired_point",1);
    solved_trajectory_pub = pnh.advertise<multidrone_msgs::SolvedTrajectory>("trajectory",1);
    path_rviz_pub = pnh.advertise<nav_msgs::Path>("solver/path",1);
    path_no_fly_zone = pnh.advertise<nav_msgs::Path>("solver/noflyzone",1);   
    target_path_rviz_pub = pnh.advertise<nav_msgs::Path>("/target/path",1);

    // pose and trajectory subscriptions
    for(int i=0; i<drones.size(); i++){ // for each drone, subscribe to the calculated trajectory and the drone pose
        drone_pose_sub[drones[i]] = pnh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drones[i])+"/ual/pose", 10, std::bind(&uavPoseCallback, std::placeholders::_1, drones[i]));               // Change for each drone ID
        if(drones[i] !=drone_id){
            drone_trajectory_sub[drones[i]] = pnh.subscribe<multidrone_msgs::SolvedTrajectory>("/drone_"+std::to_string(drones[i])+"/solver", 1, std::bind(&uavTrajectoryCallback, std::placeholders::_1, drones[i]));
        }
        //initialize
        trajectory_solved_received[drones[i]] = false;     
    }


    // log files
    csv_pose.open("/home/alfonso/trajectories"+std::to_string(drone_id)+".csv");
    //csv_record.open("/home/alfonso/to_reproduce"+std::to_string(drone_id)+".csv");
    csv_pose << std::fixed << std::setprecision(5);
    csv_record << std::fixed << std::setprecision(5);
    csv_debug.open("/home/alfonso/debug_"+std::to_string(drone_id)+".csv");

    ros::Rate rate(1); //hz
    while(!checkConnectivity()){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Solver %d is not ready",drone_id);
    }

    ROS_INFO("Solver %d is ready");
    // TODO check target pose
    
}

int main(int _argc, char **_argv)
{

    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");

    // TODO include or not calls to the uav from this node
    //go_to_waypoint_client = pnh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    // take_off_srv = pnh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    // ros::Subscriber target_pose_sub = pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
    //set_velocity_pub = pnh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);


    // init solver
    if(!init(pnh)){ // if the solver is correctly initialized
        // main loop
        while(ros::ok()){
                // solver function
                x.clear();
                y.clear();
                z.clear();
                vx.clear();
                vy.clear();
                vz.clear();
                // hardcoding for testing
                solver_success = solverFunction(x,y,z,vx,vy,vz, desired_wp, desired_vel, obst,target_vel);
                // TODO: why the definition of theses function are not here? This node should contain
                // every function that can be used with various solvers
                // logging data and visualization
                if(solver_success){
                    publishTrajectory(x,y,z,vx,vy,vz);
                }
                logToCsv(x,y,z,vx,vy,vz);
                target_path_rviz_pub.publish(targetPathVisualization()); 
                publishDesiredPoint(desired_wp[0], desired_wp[1], desired_wp[2]);
            
            ros::spinOnce();
            sleep(5);
        }
    }else{
        ros::shutdown();
        //TODO properly shutdown
    }
}
