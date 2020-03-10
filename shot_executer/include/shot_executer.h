#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <multidrone_msgs/ExecuteAction.h>
#include <multidrone_msgs/DroneAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>

/* A name class 
/**
*  The basic class description, it does:
*  1. bah
*  2. bah
*/
class ShotExecuter
{    
    public:
    ShotExecuter(ros::NodeHandle &_nh);
    protected:
        ros::NodeHandle nh;
        std::string target_topic_;
        ros::ServiceClient take_off_srv_;
        ros::ServiceClient go_to_waypoint_client_;
        ros::ServiceClient land_client_;
        ros::Publisher desired_pose_pub_;
        nav_msgs::Odometry target_pose_;
        nav_msgs::Odometry drone_pose_;
        int drone_id_;
        int step_size_;
        int time_horizon;
};

class ShotExecuterMultidrone : public ShotExecuter{
    public:
        ShotExecuterMultidrone(ros::NodeHandle &_nh);
            
    private:
        actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>* server_;
        std::thread action_thread_;
        bool goToWaypoint(const multidrone_msgs::DroneAction &_goal);
        bool takeOff(const double _height);
        std::vector<nav_msgs::Odometry> targetTrajectoryPrediction();
        void actionThread(const multidrone_msgs::DroneAction goal);
        void actionCallback();
        nav_msgs::Odometry calculateDesiredPoint(const int shooting_type, std::map<std::string, float> shooting_parameters, const std::vector<nav_msgs::Odometry> &target_trajectory);
};