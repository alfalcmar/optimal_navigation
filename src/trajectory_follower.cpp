#include <ros/ros.h>
#include <optimal_control_interface/SolvedTrajectory.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <std_srvs/SetBool.h>
#include <fstream>
#include <nav_msgs/Path.h>
#include <fstream>
#include <iostream>


std::vector<geometry_msgs::Point> velocities; //trajectory to follow
std::vector<geometry_msgs::Point> positions;  //trajectory to follow
Eigen::Vector3f current_pose;                 
Eigen::Vector3f current_vel;                 
const double look_ahead = 1.0;
int pose_on_path = 0;
int target_pose; // look ahead pose
int drone_id = 1;
bool start_trajectory = false;  //flag to start the trajectory
std::string path_csv = "/home/alfonso/traj1";
ros::Publisher csv_trajectory_pub;
const float velocity_error = 0.1;
std::ofstream csv_ual; // logging the pose
std::ofstream csv_record; // logging the pose



/** ual velocity callback **/
void ualVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
    current_vel =Eigen::Vector3f(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

/** \brief Calback for ual pose
 */
void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose = Eigen::Vector3f(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

/** \brief Callback for trayectory to follow
 */
void trajectoryCallback(const optimal_control_interface::SolvedTrajectory::ConstPtr &msg){
    ROS_INFO("Drone %d: trajectory received", drone_id);
    
    for(int i = 0; i<pose_on_path;i++){
        csv_record << positions[i].x << ", " << positions[i].y << ", " << positions[i].z<< ", "<< velocities[i].x<< ", " <<velocities[i].y<< ", " <<velocities[i].z<<std::endl;
    }
    positions.clear();
    velocities.clear();
    
    for(int i =0; i<msg->positions.size();i++){
        positions.push_back(msg->positions[i]);
        velocities.push_back(msg->velocities[i]);
    }
}

/** \brief Utility function to calculate the nearest pose on the path
 *  \param positions a path to follow
 *  \return index of the nearest pose on the path
 */
int cal_pose_on_path(const std::vector<geometry_msgs::Point> &positions, int previous_pose_on_path){
    double min_distance = 10000000;
    int pose_on_path_id = 0;
    for(int i=previous_pose_on_path; i<positions.size();i++){
        Eigen::Vector3f pose_on_path = Eigen::Vector3f(positions[i].x, positions[i].y, positions[i].z);
        if((current_pose - pose_on_path).norm()<min_distance){
            min_distance = (current_pose - pose_on_path).norm();
            pose_on_path_id = i;
        }
    }
    return pose_on_path_id;
}

/** \brief Utility function to calculate the look ahead position
 *  \param positions a path to follow
 *  \param look_ahead
 *  \pose_on_path pose from which we apply look ahead
 *  \return look ahead position index
 */

int cal_pose_look_ahead(const std::vector<geometry_msgs::Point> &positions, const double look_ahead, int pose_on_path){
    for(int i = pose_on_path; i<positions.size();i++){
        Eigen::Vector3f aux = Eigen::Vector3f(positions[i].x-positions[pose_on_path].x, positions[i].y-positions[pose_on_path].y, positions[i].z-positions[pose_on_path].z);
        double distance = aux.norm();
        if(distance>look_ahead) return i;
    }
    return positions.size();
}

/** \brief utility function to calculate velocity commands. This function apply the direction to the next point of the trajectory and the velocity of the nearest point of the trajectory.
 *  \param pose desired position of the path
 *  \param vel desired velocity of the nearest pose on the path
 *  \return 3d vector velocity to command
 */
Eigen::Vector3f calculate_vel(Eigen::Vector3f pose, Eigen::Vector3f vel){
   Eigen::Vector3f vel_to_command = (pose - current_pose).normalized();
   double vel_module = vel.norm()+velocity_error;
   /**if(vel_module<0.15){
       vel_module = 0.15;
   }
    std::cout<<vel_module<<std::endl;*/
   return vel_to_command*vel_module;
}

/**
 */

void visualizeCsvTrajectory(){
    nav_msgs::Path path_to_publish;
    path_to_publish.header.frame_id = "map";
    geometry_msgs::PoseStamped aux_pose_stamped;

    for(int i=0; i<positions.size();i++){
        aux_pose_stamped.pose.position.x = positions[i].x;
        aux_pose_stamped.pose.position.y = positions[i].y;
        aux_pose_stamped.pose.position.z = positions[i].z;
        path_to_publish.poses.push_back(aux_pose_stamped);
    }
    csv_trajectory_pub.publish(path_to_publish);
}
/** \brief callcak for start trajectory
 *  if the start trajectory is received, this callback will read trajectories from csv
 */
bool startServerCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    ROS_INFO("Drone %d: start trajectory received",drone_id);
    std::fstream read_csv;
    read_csv.open(path_csv+"_positions.csv");
    if (read_csv.is_open()) {
        while (read_csv.good()) {
            std::string x, y, z;
            double dx, dy, dz;
            getline(read_csv, x, ',');
            getline(read_csv, y, ',');
            getline(read_csv, z, '\n');
            std::stringstream sx(x);
            std::stringstream sy(y);
            std::stringstream sz(z);
            sx >> dx;
            sy >> dy;
            sz >> dz;
            geometry_msgs::Point aux_point;
            aux_point.x = dx;
            aux_point.y = dy;
            aux_point.z = dz;
            positions.push_back(aux_point);
        }
    }else{
        ROS_WARN("Follower %d: error opening csv file",drone_id);
    }
    std::fstream read_csv2;
    read_csv2.open(path_csv+"_vels.csv");
    if (read_csv2.is_open()) {
        while (read_csv2.good()) {
            std::string x, y, z;
            double dx, dy, dz;
            getline(read_csv2, x, ',');
            getline(read_csv2, y, ',');
            getline(read_csv2, z, '\n');
            std::stringstream sx(x);
            std::stringstream sy(y);
            std::stringstream sz(z);
            sx >> dx;
            sy >> dy;
            sz >> dz;
            geometry_msgs::Point aux_point;
            aux_point.x = dx;
            aux_point.y = dy;
            aux_point.z = dz;
            velocities.push_back(aux_point);
        }
    }else{
        ROS_WARN("Follower %d: error opening csv file",drone_id);
    }

    if(positions.size()==velocities.size()){
        ROS_INFO("Follower %d: trajectory has %d points",drone_id,velocities.size());
    }else
    {
        ROS_WARN("Follower %d: invalid trajectory. Discarting",drone_id);
    }
    
    visualizeCsvTrajectory();
    start_trajectory = req.data;
    res.success = true;
    res.message = "";
    return true;
}

int main(int _argc, char **_argv)
{

    ros::init(_argc, _argv, "trajectory_follower_node");
    ros::NodeHandle nh;
    ros::Subscriber trajectory_sub = nh.subscribe<optimal_control_interface::SolvedTrajectory>("solver", 1, trajectoryCallback);
    ros::Subscriber ual_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);
    ros::Subscriber ual_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("ual/velocity", 1, ualVelCallback);
    ros::Publisher velocity_ual_pub = nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
    csv_trajectory_pub =nh.advertise<nav_msgs::Path>("csv_trajectory",1);
    ros::ServiceServer start_trajectory_signal = nh.advertiseService("start_shooting",startServerCallback);
    nh.getParam("path_csv", path_csv);
   


    if (ros::param::has("~drone_id")) {
        ros::param::get("~drone_id",drone_id);
    }
    else {
        ROS_WARN("fail to get the drone id");
    }

    csv_ual.open("/home/alfonso/ual"+std::to_string(drone_id)+".csv");
    csv_record.open("/home/alfonso/record"+std::to_string(drone_id)+".csv");
    csv_record << std::fixed << std::setprecision(5);
    csv_ual << std::fixed << std::setprecision(5);

    int previous_pose_on_path = 0;
    while(ros::ok){
        ROS_INFO("Drone %d: waiting for trajectory. Pose on path: %d",drone_id,pose_on_path);
        //wait for receiving trajectories
        while((!positions.empty() && !velocities.empty())){ // if start trajectory is provided by topic or by csv
            csv_ual << current_pose[0] << ", " << current_pose[1] << ", " << current_pose[2] <<", "<< current_vel[0] << ", " << current_vel[1] << ", " << current_vel[2] << std::endl;
            pose_on_path = cal_pose_on_path(positions,previous_pose_on_path);
            previous_pose_on_path = pose_on_path;
            ROS_INFO("Drones %d: pose on path: %d", drone_id, pose_on_path);
            target_pose = cal_pose_look_ahead(positions,look_ahead, pose_on_path);
            // if the point to go is out of the trajectory, the trajectory will be finished and cleared
            if(target_pose==positions.size()){
                ROS_INFO("Drone %d: end of the trajectory",drone_id);
                positions.clear();
                velocities.clear();
                pose_on_path = 0;
                previous_pose_on_path = 0;
                break;
            }
            ROS_INFO("Drone %d: look ahead: %d",drone_id,target_pose);
            Eigen::Vector3f pose_to_go =Eigen::Vector3f(positions[target_pose].x,positions[target_pose].y, positions[target_pose].z);
            Eigen::Vector3f vel_to_go= Eigen::Vector3f(velocities[pose_on_path].x,velocities[pose_on_path].y, velocities[pose_on_path].z);
            Eigen::Vector3f velocity_to_command = calculate_vel(pose_to_go, vel_to_go);
            // publish topic to ual
            geometry_msgs::TwistStamped vel;
            vel.header.frame_id = "map";
            vel.twist.linear.x = velocity_to_command.x();
            vel.twist.linear.y = velocity_to_command.y();
            vel.twist.linear.z = velocity_to_command.z();
            velocity_ual_pub.publish(vel);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::spinOnce();
       ros::Duration(1).sleep();
    }

    return 0;
}
