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
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Eigen>
#include <math.h>       /* sqrt */
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32.h>


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
        ros::ServiceServer shooting_action_srv_;
        ros::Publisher desired_pose_pub_;
        ros::Publisher target_trajectory_pub_;
        ros::Subscriber target_pose_sub_;
        bool takeoff_called_succesfully_ = false;
        // inputs
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
        // states
        Eigen::Vector3f camera_angles_;
        const int yaw = 1;
        const int pitch = 2;

        // parameters
        int drone_id_ = 1; // TODO initialize by constructor
        int step_size_;
        const int time_horizon_= 10; //TODO read as parameter
        float rate_pose_publisher_ = 5; //Hz
        float rate_camera_publisher_ = 10; //Hz
        std::thread action_thread_;
        std::thread camera_thread_;
        std::vector<nav_msgs::Odometry> targetTrajectoryPrediction();
        nav_msgs::Odometry calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory);
        bool actionCallback(shot_executer::ShootingAction::Request  &req, shot_executer::ShootingAction::Response &res);
        void actionThread(struct shooting_action _shooting_action);
        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg);
        void calculateGimbalAngles();
        void cameraThread();
        virtual void publishCameraCommand();
};

class ShotExecuterMRS : public ShotExecuter{
    public:
        ShotExecuterMRS(ros::NodeHandle &_nh);
    private:
        ros::ServiceClient motors_client_;
        ros::ServiceClient arming_client_;
        ros::ServiceClient offboard_client_;
        ros::ServiceClient takeoff_client_;
        ros::Publisher camera_pub_;
        ros::Subscriber uav_odometry_sub;
        bool robot_armed_ = false;
        bool robot_in_offboard_mode_ = false;
        bool callTakeOff();
        bool all_motors_on_ = false;
        void publishCameraCommand();
        void uavCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

class ShotExecuterUAL : public ShotExecuter{
    public:
        ShotExecuterUAL(ros::NodeHandle &_nh);
    private:
        ros::ServiceClient take_off_srv_;
        ros::ServiceClient go_to_waypoint_client_;
        ros::ServiceClient land_client_;
        bool takeOff(const double _height);


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