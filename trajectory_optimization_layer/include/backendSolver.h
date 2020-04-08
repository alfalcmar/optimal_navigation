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
#include <tf/tf.h>
#include <formation_church_planning/Trajectory.h>
#include <formation_church_planning/Point.h>
#include <formation_church_planning/Diagnostic.h>
#include <math.h>       /* sqrt */


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



class backendSolver{
    public:
        backendSolver();
        std::vector<double> desired_pose{4,4,1,0}; 
        std::vector<double> desired_vel{0,0,0};
        ///////////////////// VARS ///////////////////////////////

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
        ros::Publisher solved_trajectory_MRS_pub;
        ros::Publisher mrs_trajectory_tracker_pub;
        ros::Publisher diagnostics_pub;
        ros::ServiceServer service_for_activation;

        std::map<int,bool> trajectory_solved_received;
        std::map<int, ros::Subscriber> drone_pose_sub;
        std::map<int, ros::Subscriber> drone_trajectory_sub;
        std::map<int, bool> has_poses; //has_poses[0] -> target
        uav_abstraction_layer::State ual_state;
        int solver_success = false;
        bool  is_initialized = false;
        ros::Timer timer;

        bool activated = false;
        bool first_activation_ = true;
        bool planning_done_ = false;
        // TODO construct a class called UAV interface and heritage methods for target pose callback and use overload depending on mrs system or us system
        // memebers that are sent to the solver must be pulic
        void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void desiredPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void uavCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id);
        void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);
        void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& _msg); // real target callback
        void publishTrajectory(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy, const std::vector<double> &vz, const std::vector<double> &yaw,const std::vector<double> &pitch);
        std::vector<double> predictingPitch(const std::vector<double> &wps_x, const std::vector<double> &wps_y, const std::vector<double> &wps_z, const std::vector<geometry_msgs::Point> &target_trajectory);
        std::vector<double> predictingYaw(const std::vector<double> &wps_x, const std::vector<double> &wps_y, const std::vector<double> &wps_z, const std::vector<geometry_msgs::Point> &target_trajectory);
        void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z);
        void publishDesiredPoint(const double x, const double y,const double z);
        nav_msgs::Path targetPathVisualization();
        void logToCsv(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::vector<double> &vx, const std::vector<double> &vy, const std::vector<double> &vz);
        void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg);
        bool checkConnectivity();
        void diagTimer(const ros::TimerEvent &event);
        bool activationServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool init(ros::NodeHandle pnh, ros::NodeHandle nh);
        void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]);
};
