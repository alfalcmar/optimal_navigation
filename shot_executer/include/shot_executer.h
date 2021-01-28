#ifndef SHOT_EXECUTER_H
#define SHOT_EXECUTER_H

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <thread>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <shot_executer/DesiredShot.h>



/**
*  \brief This node calculate the desired position to perform the desired shot. It is calculated from the state of the target and the shot requested by the user.
*         Input: 
*               - shooting action request
*               - target state
*         Outpu: 
*               - Desired pose
*/
class ShotExecuter
{    
    public:
        /** \brief Generic constructor of the class. This constructor reads generic parameters and subscribes to topics.
         *  \param _nh public nodehandle
         */
        ShotExecuter(ros::NodeHandle &_nh, ros::NodeHandle &_pnh, std::string frame);
    protected:

        //ROS (publishers, subscribers)
        std::string target_topic_;  /*< Target topic */
        ros::ServiceServer shooting_action_srv_; /*< Server to choose the action */
        ros::Publisher desired_pose_pub_;        /*< That publish the desired pose */
        ros::Publisher target_trajectory_pub_;   /*< That publish the target trajectory prediction */
        ros::Subscriber target_pose_sub_;       /*< Subscribe to target pose */
        bool takeoff_called_succesfully_ = false;   /*< Flag for take off */
        ros::Publisher desired_pose_publisher; /**< desired pose publisher for RVIZ visualization */
        const std::string frame_;
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
        const int ROLL = 0;
        const int YAW = 1;
        const int PITCH = 2;
        const float MIN_XY_VEL = 0.4;
        double target_orientation_[3];
        int prediction_mode_ = 0; /**< to predict the direction of the target using target velocity (velocity mode = 0) or target orientation (orientation mode = 1)*/
        const int VELOCITY_MODE = 0;    /**< trajectory predicted using the target velocity */
        const int ORIENTATION_MODE = 1; /**< trajectory predicted using the target orientation */
        const double VEL_CTE = 0.5; /**< Target velocity guess to predict trajectory */
        const float MAX_DRONE_VEL = 1;
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

        void calculateHorizonPoint(nav_msgs::Odometry &desired_point, const nav_msgs::Odometry &drone_pose);
        /** \brief This predicts the target trajectory over time_horizon_. We use a velocity cte model.
         *          Besides, publish the predicted trajectory for visualization
         *  \return The predicted target trajectory 
         */
        std::vector<nav_msgs::Odometry> targetTrajectoryPrediction();

        /** \brief Calculate desired pose. If type flyby, calculate wrt last mission pose . If lateral, calculate wrt time horizon pose
         *  \TODO   z position and velocity, angle relative to target
         *  \TODO   calculate orientation by velocity and apply it to desired pose
         */
        shot_executer::DesiredShot calculateDesiredPoint(const struct shooting_action _shooting_action, const std::vector<nav_msgs::Odometry> &target_trajectory);
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
        /** \brief publish desired point to rviz
         */
        void publishDesiredPoint(nav_msgs::Odometry desired_odometry);
};

#endif
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

