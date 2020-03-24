#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <thread>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>
#include <shot_executer/ShootingAction.h>
#include <geometry_msgs/Vector3.h>
/* A name class 
/**
*  The basic class description, it does:
*  TODO: target velocity
*  TODO
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
        ros::ServiceServer shooting_action_srv_;
        ros::Publisher desired_pose_pub_;
        ros::Publisher target_trajectory_pub_;
        ros::Subscriber target_pose_sub_;
        nav_msgs::Odometry target_pose_;
        nav_msgs::Odometry drone_pose_;
        //shooting action fields
        struct shooting_action{
            std::string start_event = "";
            int shooting_action_type = 0;
            double duration = 0.0;
            double length = 0.0;
            int target_type;
            geometry_msgs::Vector3 rt_parameters; 
        };
        int drone_id_;
        int step_size_;
        const int time_horizon_= 10; //TODO read as parameter
        double rate_pose_publisher_ = 1.0;
        std::thread action_thread_;
        bool takeOff(const double _height);
        std::vector<nav_msgs::Odometry> targetTrajectoryPrediction();
        nav_msgs::Odometry calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory);
        bool actionCallback(shot_executer::ShootingAction::Request  &req, shot_executer::ShootingAction::Response &res);
        void actionThread(struct shooting_action _shooting_action);
        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);

};
#ifdef MULTIDRONE
class ShotExecuterMultidrone : public ShotExecuter{
    public:
        ShotExecuterMultidrone(ros::NodeHandle &_nh);
            
    private:
        actionlib::SimpleActionServer<multidrone_msgs::ExecuteAction>* server_;
        std::thread action_thread_;
        bool goToWaypoint(const multidrone_msgs::DroneAction &_goal);
        void actionThread(const multidrone_msgs::DroneAction goal);
};
#endif