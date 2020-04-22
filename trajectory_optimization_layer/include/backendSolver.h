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
#include <ACADO.h>


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
        backendSolver(ros::NodeHandle pnh, ros::NodeHandle nh);
    protected:

        // solver output - state variables - position and velocities (ROBOT) change to array
        std::vector<double> x_;  
        std::vector<double> y_;
        std::vector<double> z_;
        std::vector<double> vx_;
        std::vector<double> vy_;
        std::vector<double> vz_;
        //robots
        int drone_id_;
        std::map<int,nav_msgs::Odometry> uavs_pose_;                      /**< Last uavs odometry <drone_id,odometry> */
        std::map<int,optimal_control_interface::Solver> uavs_trajectory;  /**< Last trajectory solved by others <drone_id,odometry*/
        //target
        nav_msgs::Odometry target_odometry_;                 /**< Last target odometry */
        std::vector<nav_msgs::Odometry> target_trajectory_;  /**< Predicted target trajetory*/
        // no fly zone
        std::array<float,2> obst_{0.0,0.0};             /**< No fly zone (infinite cylinder) [x y]*/
        // desired pose
        const double REACHING_TOLERANCE = 2.0;      /**< Distance to the desired pose that is set as reached */
        nav_msgs::Odometry desired_odometry_;       /**< Desired pose [x y z yaw] */ 
        //solver
        int time_horizon_ = 39;                     /**< Number of steps */
        double solver_rate_ = 4;                    /**< Rate to call the solver (s) */
        int solver_success = false;                 /**< the solver has solved successfully */
        bool multi_ = false;                        /**< true if multi uav formation is activated */
        bool target_ = true;                        /**< true if there is a target that is being filmed*/
        std::vector<int> drones;
        // services and topics
        ros::Subscriber target_array_sub;           /**< Subscriber to target topic */
        ros::Subscriber desired_pose_sub;           /**< Subscriber to Shot executer's desired pose*/
        ros::Publisher solved_trajectory_pub;       /**< Publisher for the solve trajectroy for others */
        ros::ServiceServer service_for_activation;  /**< service to activate the planning */
        ros::Publisher path_rviz_pub;               /**< Publisher for visualizing the generated trajectory on RVIZ */            
        ros::Publisher target_path_rviz_pub;        /**< Publisher for visualizing the target trajectory on RVIZ */     
        ros::Publisher path_no_fly_zone;            /**< Publisher for visualizing the no-fly zone RVIZ */     
        ros::Publisher desired_pose_publisher;      /**< desired pose publisher for RVIZ visualization */
        std::map<int, ros::Subscriber> drone_pose_sub;  /**< subscribers of the drones poses <drone_id, pose_subscriber> */
        std::map<int, ros::Subscriber> drone_trajectory_sub;    /**< subscribers the solved trajectory of others <drone_id, trajectory_subscriber */
        // aux flags
        std::map<int, bool> has_poses;                   /**< map to register if the poses of the robots have been received <drone_id, received> drone_id = 0 -> target */
        std::map<int,bool> trajectory_solved_received;   /**<  flags to receive the solved trajectories for others <drone_id, received> */
        bool is_initialized = false;            /**< object inizialized */
        bool activated = false;                 /**< planning activated */
        bool first_activation_ = true;          /**< first activation of the planning */
        bool planning_done_ = false;            /**< planning finished */
        // timers and threads
        ros::Timer diagnostic_timer_;           /**< timer to publish diagnostic topic */
        std::thread main_thread_;                   /**< Main thread that calls the solver*/

        const int TARGET = 0;                   /**< has_poses[TARGET] */

        ACADOsolver acado_solver_;

        ///////////////// CALLBACKS MEMBERS ///////////////////////////////
        /*!  \brief callback for the desired pose. This function receives the pose in quaternion
         *   \param msg
         *   \TODO check yaw, not being save anymore
         */
        void desiredPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);
        
        /*!  \brief Callback 
         *   \param msg solved trajectory of others saved in uavs_trajectory[drone_id]
         *   \param id drone_id 
         */
        void uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id);
        /*!  \brief receives uav's pose and save it in uavs_pose[id]. If the solved trajectory from other is not received, it initializa solved_trajectory with this pose
         *   \param msg uav's pose
         *   \param id  drone_id
         */
        void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);
   
        //////////////// UTILITY FUNCTION ////////////////////////////////
        /** \brief This function publish the calculated trajectory to be read by other drones
        *  \param x y z vx vy vz    last calculated trajectory
        *          void publishTrajectory(const std::vector<double> &yaw,const std::vector<double> &pitch);
        */
        void publishTrajectory();
        /*! \brief This function calculates the pitch of the camera over the N steps through calculated trajectroy and target trajectory
        *   \return Predicted pitch 
        **/
        std::vector<double> predictingPitch();
        /*! \brief This function calculates the yaw of the drone over the N steps (pointing to the target) through calculated trajectory and target trajectory
        *   \return Predicted yaw over N steps
        **/
        std::vector<double> predictingYaw();

        /** \brief Utility function to predict the trajectory of the target along the N steps. This function use a velocity cte model to predict the target trajectory
        *  \TODO Predict target trajectory with actual pose and velocity
        */
        void targetTrajectoryVelocityCTEModel();
        /** \brief Utility function to get quaternion from pitch, roll, yaw
         *  \param pitch
         *  \param roll
         *  \param yaw
         *  \return quaternion transformation
        */
        geometry_msgs::Quaternion toQuaternion(const double pitch, const double roll, const double yaw);

        /*! \brief This function checks that the pose of others and the target are being received
        *   \return true if connectivity is correct
        **/
        bool checkConnectivity();
        /*! \brief Service callback to activate the planning
        *   \param req req.data = true -> activate planning, req.data = false -> deactivate planning
        *   \param res res.message = "message"
        *   \return planning_activated 
        **/
        bool activationServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        /*! \brief If the planning is active: clean the state variables, call solver function, predict yaw and pitch, publish solved trajectories, publish data to visualize 
        **/
        virtual void callSolverLoop();
        /** \brief Utility function to calculate if the trajectory calculated by the solver finishes in the desired pose
        *  \param desired_pos      This is the desired pose
        *  \param last_traj_pos    This is the last point of the calculated trajectory
        *  \return                 it will return true if the calculated trajectory reach the desired position
        */
        bool desiredPoseReached(const std::vector<double> desired_pos, const std::vector<double> last_traj_pos);
        ///////////////////////// UTILITY FUNCTION FOR VISUALIZATION AND LOGGING ////////////////////////////
        /*! \brief Publish solved path to visualize on RVIZ
        *   \param wps solved path
        **/
        void publishPath();
        /*! \brief Publish desired pose to visualize on RVIZ
        **/
        void publishDesiredPoint();
        /*! \brief Utility function to create a msg that allows us to visualize the target path prediction on RVIZ
        *   \return nav_msg to visualize
        **/
        nav_msgs::Path targetPathVisualization();
        /*! \brief   This function log the solved trajectory into a csv file
        *   \param xyz 3D trajectory
        **/
        void logToCsv();

        /*! \brief Publish a rectangle that represents a no fly zone in order to visualize on rviz
        *   \param point_1 2D vertice
        *   \param point_2 2D vertice
        *    
        **/
        void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]);

};

class backendSolverMRS : backendSolver {
    public:
        backendSolverMRS(ros::NodeHandle &_pnh, ros::NodeHandle &_nh);
    private:
        ros::Subscriber uav_odometry_sub;           /**< Subscriber to UAV's odometry*/
        ros::Publisher diagnostics_pub;
        ros::Publisher solved_trajectory_MRS_pub;
        ros::Publisher mrs_trajectory_tracker_pub;
        /*! \brief Callback function to the target topic. This function save the pose reveived for the first in has_poses[0] and upload target_pose_ member
         *  \param 
         * */
        void targetCallbackMRS(const geometry_msgs::PoseStamped::ConstPtr& _msg);
        
        void uavCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void publishSolvedTrajectory(const std::vector<double> &yaw,const std::vector<double> &pitch);
        void diagTimer(const ros::TimerEvent &event);
        void callSolverLoop();


};


class backendSolverUAL : backendSolver {
    public:
        backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh); /**< UAL backend constructor*/
    private:
        ros::Subscriber uav_state_sub;              /**< Subscriber to UAL's state*/
        ros::Subscriber sub_velocity;               /**< Subscriber to UAL's velocity*/
        uav_abstraction_layer::State ual_state_;     /**< ual state */

        /*!  \brief callback that save the ual velocity
         *   \param msg
         */
        void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg); /**< Callback for UAL's state topic */
        void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg);   /**< Callback for UAL's velocity topic*/
        /*!  \brief target pose topic callback. This function save the first time received and the msg into target_pose_
         *   \param msg topic msg
         */
        void targetPoseCallbackGRVC(const nav_msgs::Odometry::ConstPtr &msg);

};