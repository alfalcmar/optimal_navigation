#include <ros/ros.h>
#include <optimal_control_interface/SolvedTrajectory.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/GoToWaypoint.h>


std::vector<geometry_msgs::Point> velocities(400);
std::vector<geometry_msgs::Point> positions(400);
Eigen::Vector3f current_pose;
const double look_ahead = 1.0;
ros::ServiceClient go_to_waypoint_client;
int pose_on_path;
int target_pose;
int drone_id = 1;


void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_pose = Eigen::Vector3f(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}
void trajectoryCallback(const optimal_control_interface::SolvedTrajectory::ConstPtr &msg){
    ROS_INFO("Drone %d: trajectory received", drone_id);
    pose_on_path = 0;
    target_pose = 0;
    positions = msg->positions;
    velocities = msg->velocities;

    /*uav_abstraction_layer::GoToWaypoint srv;
    srv.request.blocking = true;
    srv.request.waypoint.position.pose.x = positions.at(0).x;
    srv.request.waypoint.position.pose.y = positions.at(0).y;
    srv.request.waypoint.position.pose.z = positions.at(0).z;
    if(go_to_waypoint_client.call(srv)){
        ROS_INFO("Drone %d: going to initial pose");
    }else{
        ROS_WARN("Go to initial pose failed, discarting trajectory");
    }*/
}

// this function calculate the nearest pose on the path
int cal_pose_on_path(const std::vector<geometry_msgs::Point> &positions){
    double min_distance = 10000000;
    int pose_on_path_id = 0;
    for(int i=0; i<positions.size();i++){
        Eigen::Vector3f pose_on_path = Eigen::Vector3f(positions[i].x, positions[i].y, positions[i].z);
        if((current_pose - pose_on_path).norm()<min_distance){
            min_distance = (current_pose - pose_on_path).norm();
            pose_on_path_id = i;
        }
    }
    return pose_on_path_id;
}

int cal_pose_look_ahead(const std::vector<geometry_msgs::Point> &positions, double look_ahead, int pose_on_path){
    for(int i = pose_on_path; i<positions.size();i++){
        Eigen::Vector3f aux = Eigen::Vector3f(positions[i].x-positions[pose_on_path].x, positions[i].y-positions[pose_on_path].y, positions[i].z-positions[pose_on_path].z);
        double distance = aux.norm();
        if(distance>look_ahead) return i;
    }
    return positions.size();
}

Eigen::Vector3f calculate_vel(Eigen::Vector3f pose, Eigen::Vector3f vel){
   Eigen::Vector3f vel_to_command = (pose - current_pose).normalized();
   double vel_module = vel.norm();
   return vel_to_command*vel_module;
}

int main(int _argc, char **_argv)
{

    pose_on_path = positions.size();
    target_pose = positions.size();
    // press a key to start following the trajectory

    ros::init(_argc, _argv, "trajectory_follower_node");
    ros::NodeHandle nh;
    ros::Subscriber trajectory_sub;
    ros::Publisher velocity_ual_pub;
    trajectory_sub = nh.subscribe<optimal_control_interface::SolvedTrajectory>("solver", 1, trajectoryCallback);
    ros::Subscriber ual_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);
    go_to_waypoint_client = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");

    velocity_ual_pub = nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
    double control_rate = 10.0;
    double height_take_off = 3.0;
    //nh.param<float>("control_rate", control_rate, 1.0);
    //nh.param<float>("height_take_off", height_take_off, 4.0);

    const int hover_final_pose = 5;
    // taking off
    ros::ServiceClient take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = height_take_off;
    if(!take_off_srv.call(srv)){
        ROS_WARN("the take off is not available");
    }else{
        ROS_INFO("taking_off");
    }


    while(ros::ok){
        ROS_INFO("Drone %d: waiting for trajectory. Pose on path: %d",drone_id,pose_on_path);
        while(pose_on_path!=positions.size()){
            pose_on_path = cal_pose_on_path(positions);
            ROS_INFO("pose on path: %d", pose_on_path);
            target_pose = cal_pose_look_ahead(positions,look_ahead, pose_on_path);
            if(target_pose==positions.size()){
                ROS_INFO("Drone %d: end of the trajectory",drone_id);
                pose_on_path = positions.size();
                break;
            }
            ROS_INFO("look ahead: %d",target_pose);
            Eigen::Vector3f pose_to_go =Eigen::Vector3f(positions[target_pose].x,positions[target_pose].y, positions[target_pose].z);
            Eigen::Vector3f vel_to_go= Eigen::Vector3f(velocities[pose_on_path].x,velocities[pose_on_path].y, velocities[pose_on_path].z);
            Eigen::Vector3f velocity_to_command = calculate_vel(pose_to_go, vel_to_go);
            geometry_msgs::TwistStamped vel;
            vel.twist.linear.x = velocity_to_command.x();
            vel.twist.linear.y = velocity_to_command.y();
            vel.twist.linear.z = velocity_to_command.z();
            velocity_ual_pub.publish(vel);
            ros::spinOnce();
        }
        ros::spinOnce();
        sleep(1);
    }

    return 0;
}

// cada vez que una trayectoria sea recibida, resetear