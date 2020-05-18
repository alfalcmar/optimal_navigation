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
#include <tf/tf.h>



/**
*  \brief The basic class description, it does:
*/
class ShotExecuter
{    
    public:
        /** \brief Generic constructor of the class. This constructor reads generic parameters and subscribes to topics.
         *  \param _nh public nodehandle
         */
        ShotExecuter(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);
    protected:

        //ROS (publishers, subscribers)
        ros::NodeHandle nh;   /*< Nodehandle */
        std::string target_topic_;  /*< Target topic */
        ros::ServiceServer shooting_action_srv_; /*< Server to choose the action */
        ros::Publisher desired_pose_pub_;        /*< That publish the desired pose */
        ros::Publisher target_trajectory_pub_;   /*< That publish the target trajectory prediction */
        ros::Subscriber target_pose_sub_;       /*< Subscribe to target pose */
        bool takeoff_called_succesfully_ = false;   /*< Flag for take off */

        // inputs
        nav_msgs::Odometry target_pose_;            /*< target pose */
        nav_msgs::Odometry drone_pose_;             /*< drone pose*/

        /* Struct to define the shooting action
        */
        struct shooting_action{
            std::string start_event = "";
            int shooting_action_type = 0;
            double duration = 0.0;
            double length = 0.0;
            int target_type;
            geometry_msgs::Vector3 rt_parameters; 
        };
        Eigen::Vector3f camera_angles_; /**< member to save the angles of the camera to point the target. It is calculate by calculateGimbalAngles*/
        const int YAW = 1;
        const int PITCH = 2;

        int prediction_mode_ = 0; /**< to predict the direction of the target using target velocity (velocity mode = 0) or target orientation (orientation mode = 1)*/
        const int VELOCITY_MODE = 0;
        const int ORIENTATION_MODE = 1;
        const double VEL_CTE = 0.5;

        // parameters
        int drone_id_ = 1; // TODO initialize by constructor
        const float step_size_ = 0.2;    /**< step size (seconds) */
        const int time_horizon_= 40; /*< Number of steps */ 
        float rate_pose_publisher_ = 5; /**< Rate to publish the desired pose (Hz) */
        float rate_camera_publisher_ = 10; /**< Rate to publish the camera pose (Hz) */
        std::thread action_thread_;  /*< thread that publish the desired pose */
        std::thread camera_thread_;  /*< thread that publish the pith and yaw needed for the camera pointing to the target */
        
        bool new_shooting_action_received_ = false;
        bool shooting_action_running_ = false;
        /** \brief This predicts the target trajectory over time_horizon_. We use a velocity cte model.
         *          Besides, publish the predicted trajectory for visualization
         *  \return The predicted target trajectory 
         */
        std::vector<nav_msgs::Odometry> targetTrajectoryPrediction();

        /** \brief Calculate desired pose. If type flyby, calculate wrt last mission pose . If lateral, calculate wrt time horizon pose
         *  \TODO   z position and velocity, angle relative to target
         *  \TODO   calculate orientation by velocity and apply it to desired pose
         */
        nav_msgs::Odometry calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory);
        /** \brief Callback for action service. Receive the request and start a thread with this request actionThread()
         *         If there are any shooting action active, it command it to finish and wait it to finish
         */
        bool actionCallback(shot_executer::ShootingAction::Request  &req, shot_executer::ShootingAction::Response &res);
        /** \brief This function predicts the target trajectory and calculates the desired pose while the shooting action lasts or while a new one is received.
         *  \param _shooting_action shooting action to execute 
         *  \TODO implement time_reached
         *  \TODO implement distance_reached
         */
        void actionThread(struct shooting_action _shooting_action);
        /** \brief Callback for the target pose
         */
        void targetPoseCallback(const nav_msgs::Odometry::ConstPtr& _msg);
        /** \brief This function calculates the camera angles needed to point to the target.
         *         It is saved in camera_angles_ member
         */
        void calculateGimbalAngles();
        /** This thread is publishing camera_angles_ every time.
         */
        void cameraThread();
        /** \brief prototype function to call the camera topic
         */
        virtual void publishCameraCommand();
};

/**
 * \TODO: target velocity is set as 0

 */
class ShotExecuterMRS : public ShotExecuter{
    public:
        ShotExecuterMRS(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);
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
        ShotExecuterUAL(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);
    private:
        ros::ServiceClient take_off_srv_;
        ros::ServiceClient go_to_waypoint_client_;
        ros::ServiceClient land_client_;
        /** \brief Taking off the drone
        *  \param _height   take off's height
        *  \return success
        **/
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