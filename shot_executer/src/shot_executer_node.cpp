#include <shot_executer.h>

/** \brief main function of the shot executer node
*/

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "Shot Executer");
    ros::NodeHandle nh;


    std::string target_topic;
    nh.param<std::string>("target_topic",target_topic, "/drc_vehicle_xp900/odometry");

    ShotExecuter shotExecuter(nh);
//     // subscribers and publishers
//     uav_state_sub = nh.subscribe<uav_abstraction_layer::State>("ual/state",1,ualStateCallback);
//     go_to_waypoint_client = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("ual/go_to_waypoint");
//     take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("ual/take_off");
//    // ros::Subscriber target_pose_sub = nh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
//     ros::Subscriber target_array_sub = nh.subscribe<multidrone_msgs::TargetStateArray>("/target_3d_state", 1, targetarrayCallback);

//     set_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity",1);
    ros::spin();
    return 0;
}
