#include "FORCESNLPsolver.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <FORCES_PRO.h>
#include <mrs_msgs/TrackerTrajectory.h>
#include <mrs_msgs/TrackerPoint.h>
#include <std_srvs/SetBool.h>

/** this node is a backend to use optimization solvers with ROS for UAVs, 
 * In this case, this use the FORCES_PRO librarly, a library created to use the FORCES PRO framework
 * to interface with the rest of the nodes.
 * In summary this node contains the following:
 * Callback for interfacing with the rest of the nodes:
 *     Callback for other's calculated trajectories
 *     Callback for Drones's pose
 *     Callback for Target Pose
 *     Callback for the desired pose (In this case is received from the shot executer)
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
 *     uavs_trajectory[id, solver]
 *     uavs_poses[id, pose]
 *     target_pose
 *     target_trajectory[]
 * **/



///////////////////// VARS ///////////////////////////////

std::vector<double> desired_pose{4,4,1}; 
std::vector<double> desired_vel{0,0,0};
std::vector<double> obst{0,0};  //TODO change to map
std::vector<double> target_vel = {0, 0};
float solver_rate;
ros::Subscriber uav_odometry_sub;
ros::Subscriber uav_state_sub;
ros::Subscriber target_array_sub;
ros::Subscriber sub_velocity;
ros::Subscriber desired_pose_sub;
ros::Publisher path_rviz_pub;
ros::Publisher target_path_rviz_pub;
ros::Publisher path_no_fly_zone;
ros::Publisher desired_pose_publisher;
ros::Publisher solved_trajectory_pub;
ros::Publisher mrs_trajectory_tracker_pub;

std::map<int,bool> trajectory_solved_received;
std::map<int, ros::Subscriber> drone_pose_sub;
std::map<int, ros::Subscriber> drone_trajectory_sub;
std::map<int, bool> has_poses; //has_poses[0] -> target
uav_abstraction_layer::State ual_state;
int solver_success;


// TODO construct a class called UAV interface and heritage methods for target pose callback and use overload depending on mrs system or us system
// memebers that are sent to the solver must be pulic





///////////////////// Callbacks //////////////////////////


/** \brief Callback for the target pose
 */
void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    has_poses[0] = true;
    target_pose.pose = msg->pose.pose;
}

/** \brief Callback for the desired pose (provided by shot executer)
 */
void desiredPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    desired_pose[0] = msg->pose.pose.position.x;
    desired_pose[1] = msg->pose.pose.position.y;
    desired_pose[2] = height;
    ROS_INFO("Desired pose received: x: %f y: %f z: %f",desired_pose[0],desired_pose[1],desired_pose[2]);
}

/** \brief uav odometry callback (mrs system)
 */
void uavCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    own_velocity.twist = msg->twist.twist;
    uavs_pose[drone_id].pose = msg->pose.pose;
    has_poses[drone_id] = true; 
}

/** \brief Drone velocity callback
 */
void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    own_velocity.twist = msg->twist;
}

/** \brief This callback receives the solved trajectory of uavs
 */
void uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id){
    uavs_trajectory[id].positions.clear();
    uavs_trajectory[id].velocities.clear();
    uavs_trajectory[id].accelerations.clear();
    trajectory_solved_received[id] = true;
    uavs_trajectory[id].positions = msg->positions;
    uavs_trajectory[id].velocities = msg->velocities;
    uavs_trajectory[id].accelerations = msg->accelerations;
    ROS_INFO("Solver %d: trajectory callback from drone %d",drone_id,id);
}

/** \brief callback for the pose of uavs
 */
void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id){
    has_poses[id] = true;
    if(!trajectory_solved_received[id]){
        for(int i=0; i<time_horizon;i++){
            geometry_msgs::PoseStamped pose_aux;
            pose_aux.pose.position.x = msg->pose.position.x;
            pose_aux.pose.position.y = msg->pose.position.y;
            pose_aux.pose.position.z = msg->pose.position.z;
            uavs_trajectory[id].positions.push_back(pose_aux);
        }
    }
    uavs_pose[id].pose = msg->pose; 
}


/** \brief target pose callback
 */
void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg) // real target callback
{
    has_poses[0] = true;

    target_pose = *_msg;
}

//////////////////////// Visualization function //////////////////////////

/** \brief This function publish the calculated trajectory to be read by other drones
 *  \param x y z vx vy vz       last calculated trajectory
*/
void publishTrajectory(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy, const std::vector<double> &vz){
    
    optimal_control_interface::Solver traj;
    geometry_msgs::PoseStamped pos;
    geometry_msgs::Twist vel;

    mrs_msgs::TrackerPoint aux_point;
    mrs_msgs::TrackerTrajectory traj_to_command;
    traj_to_command.fly_now = true;
    for(int i=0;i<x.size(); i++){
        pos.pose.position.x = x[i];
        pos.pose.position.y = y[i];
        pos.pose.position.z = z[i];
        traj.positions.push_back(pos);
        vel.linear.x = vx[i];
        vel.linear.y = vy[i];
        vel.linear.z = vz[i];
        traj.velocities.push_back(vel);

        aux_point.x = x[i];
        aux_point.y = y[i];
        aux_point.z = z[i];
        traj_to_command.points.push_back(aux_point);
    }   
    mrs_trajectory_tracker_pub.publish(traj_to_command);
    solved_trajectory_pub.publish(traj);
}
/**  \brief Construct a nav_msgs_path and publish to visualize through rviz
 *   \param wps_x, wps_y, wps_z     last calculated path
 */

void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z) {
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    msg.header.frame_id = "uav1/gps_origin";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
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
void publishDesiredPoint(const double x, const double y,const double z){
    geometry_msgs::PointStamped desired_point;
    desired_point.point.x = x;
    desired_point.point.y = y;
    desired_point.point.z = z;
    desired_point.header.frame_id = "uav1/gps_origin";

    desired_pose_publisher.publish(desired_point);
}

/** \brief Construct the no fly zone approximated by a tetrahedron to visualize it on RVIZ 
 *  \param  2D points
*/
void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]){
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


/** \brief function for logging the calculated trajectory to csv file
 *  \param x,y,z,vx,vy,vz        calculated trajectory
 */
void logToCsv(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy, const std::vector<double> &vz){
    // logging all results
    csv_pose<<std::endl;
     for(int i=0; i<x.size(); i++){
        csv_pose << x[i] << ", " << y[i] << ", " << z[i]<< ", "<< vx[i]<< ", " <<vy[i]<< ", " <<vz[i]<<std::endl;
    }
}


/** \brief function to visualize the predicted target path
 */
nav_msgs::Path targetPathVisualization()
{
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(target_trajectory.size());
    msg.header.frame_id = "uav1/gps_origin";
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

/** \brief callback for ual state
 *  \TODO use or remove
 */

void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg){
    ual_state.state = msg->state;
}


 //////////////////// utility function ///////////////////////////////////

/** \brief Function to check connectivity between nodes
 * \TODO check target pose
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


/** \brief Function to initialize the solver. This function heck if the drone is subscribed to the others drone poses and target**/
bool init(ros::NodeHandle pnh, ros::NodeHandle nh){
    // parameters
    pnh.param<float>("solver_rate", solver_rate, 0.5); // solver rate
    std::string target_topic;
    pnh.param<std::string>("target_topic",target_topic, "/gazebo/dynamic_target/dynamic_pickup/pose"); // target topic
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
    //sub_velocity = pnh.subscribe<geometry_msgs::TwistStamped>("ual/velocity",1,ownVelocityCallback);
    //uav_state_sub = pnh.subscribe<uav_abstraction_layer::State>("ual/state",1,ualStateCallback); //ual state
    uav_odometry_sub = nh.subscribe<nav_msgs::Odometry>("odometry/odom_main",1, uavCallback);
    target_array_sub = pnh.subscribe<geometry_msgs::PoseStamped>(target_topic, 1, targetCallback); //target pose
    desired_pose_sub = nh.subscribe<nav_msgs::Odometry>("desired_pose",1,desiredPoseCallback); // desired pose from shot executer
    // publishers
    mrs_trajectory_tracker_pub = nh.advertise<mrs_msgs::TrackerTrajectory>("control_manager/mpc_tracker/set_trajectory",1);
    desired_pose_publisher = pnh.advertise<geometry_msgs::PointStamped>("desired_point",1);
    solved_trajectory_pub = pnh.advertise<optimal_control_interface::Solver>("trajectory",1);
    path_rviz_pub = pnh.advertise<nav_msgs::Path>("path",1);
    path_no_fly_zone = pnh.advertise<nav_msgs::Path>("noflyzone",1);   
    target_path_rviz_pub = pnh.advertise<nav_msgs::Path>("target/path",1);

    // TODO integrate it into the class UAL interface
    // pose and trajectory subscriptions
   /** for(int i=0; i<drones.size(); i++){ // for each drone, subscribe to the calculated trajectory and the drone pose
        drone_pose_sub[drones[i]] = pnh.subscribe<geometry_msgs::PoseStamped>("/drone_"+std::to_string(drones[i])+"/ual/pose", 10, std::bind(&uavPoseCallback, std::placeholders::_1, drones[i]));               // Change for each drone ID
        if(drones[i] !=drone_id){
            drone_trajectory_sub[drones[i]] = pnh.subscribe<optimal_control_interface::Solver>("/drone_"+std::to_string(drones[i])+"/solver", 1, std::bind(&uavTrajectoryCallback, std::placeholders::_1, drones[i]));
        }
        //initialize
        trajectory_solved_received[drones[i]] = false;     
    }*/


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

    ROS_INFO("Solver %d is ready", drone_id);
    // TODO check target pose
    
}

int main(int _argc, char **_argv)
{

    // ros node initialization
    ros::init(_argc, _argv,"solver");
    ros::NodeHandle pnh = ros::NodeHandle("~");
    ros::NodeHandle nh;

    // TODO include or not calls to the uav from this node
    //go_to_waypoint_client = pnh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
    // take_off_srv = pnh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    // ros::Subscriber target_pose_sub = pnh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
    //set_velocity_pub = pnh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);


    // init solver
    if(!init(pnh, nh)){ // if the solver is correctly initialized
        // main loop
        while(ros::ok()){
            // solver function
            x.clear();
            y.clear();
            z.clear();
            vx.clear();
            vy.clear();
            vz.clear();
            // call the solver function
            solver_success = solverFunction(x,y,z,vx,vy,vz, desired_pose, desired_vel, obst,target_vel);
            // TODO: why the definition of theses function are not here? This node should contain
            // every function that can be used with various solvers
            if(solver_success){
                publishTrajectory(x,y,z,vx,vy,vz);
            }
            logToCsv(x,y,z,vx,vy,vz);
            target_path_rviz_pub.publish(targetPathVisualization()); 
            publishDesiredPoint(desired_pose[0], desired_pose[1], desired_pose[2]);
            publishPath(x,y,z);
            ros::spinOnce();
            sleep(5);
        }
    }else{
        ros::shutdown();
    }
}
