#ifndef BACKENDSOLVER_H
#define BACKENDSOLVER_H
#ifdef FORCES
#include "FORCESNLPsolver.h"
#include <FORCES_PRO.h>
#endif
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <tf/tf.h>
#include <math.h> /* sqrt */
#include <solver_acado.h>
#include <shot_executer/DesiredShot.h>
#include <optimal_control_interface/Solver.h>
#include <thread>  // std::thread, std::this_thread::sleep_for
#include <ros/package.h>
#include <chrono>
#include <UAVState.h>

// MRS headers, TODO: avoid in this class
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/String.h>
#include <algorithm>
#define ZERO 0.000001


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
namespace SolverUtils{ //forward declarationX
class Logger;
}
class backendSolver {
public:
  backendSolver(ros::NodeHandle pnh, ros::NodeHandle nh, int time_horizon);
    /*! \brief If the planning is active: clean the state variables, call solver function, predict yaw and pitch, publish solved trajectories, publish data to
    *visualize
   **/
  void stateMachine();


protected:
  // solver output - state variables - position and velocities (ROBOT) change to array
  const int time_horizon_;

  std::string         pcd_file_path_;  // path to pcd file containing representation of obstacles
  std::unique_ptr<State[]> solution_;  
  

  std::shared_ptr<State[]> initial_guess_;

  std::shared_ptr<safe_corridor_generator::SafeCorridorGenerator>  safe_corridor_generator_;


  std::vector<float>                no_fly_zone_center_;
  const float                       NO_FLY_ZONE_RADIUS = 4;
  std::vector<std::array<float, 2>> no_fly_zone_points_;
  const float                       max_vel                = 1.0; /**< Max velocity imposed as constraint */
  const float                       diagnostic_timer_rate_ = 0.5; /**< rate of the diagnostic timer for MRS system (s) */
  // robots
  int                                              drone_id_;
  std::map<int, UavState>                uavs_pose_;      /**< Last uavs odometry <drone_id,odometry> */
  std::map<int, optimal_control_interface::Solver> uavs_trajectory; /**< Last trajectory solved by others <drone_id,odometry*/
  // target
  nav_msgs::Odometry              target_odometry_;   /**< Last target odometry */
  std::vector<nav_msgs::Odometry> target_trajectory_; /**< Predicted target trajetory*/
  // no fly zone
  std::array<float, 2> obst_{0.0, 0.0}; /**< No fly zone (infinite cylinder) [x y]*/
  // desired pose
  const double       REACHING_TOLERANCE = 2.0; /**< Distance to the desired pose that is set as reached */
  nav_msgs::Odometry desired_odometry_;        /**< Desired pose [x y z yaw] */
  int                desired_type_ = shot_executer::DesiredShot::IDLE;
  // solver
  float        solver_rate_        = 0.5; /**< Rate to call the solver (Hz) */
  int          solver_success      = -1;                                        /**< the solver has solved successfully */
  bool         multi_              = false;                                     /**< true if multi uav formation is activated */
  bool         target_             = true;                                      /**< true if there is a target that is being filmed*/
  const double step_size           = 0.2;                                       /**< step size (seg) */
  bool         first_time_solving_ = true;
  bool         height_reached_     = false; /**< utility flag to set true when the height of the shot is reached */

  const float OFFSET_YAW = -0.5;

  std::vector<int> drones;
  // services and topics
  ros::Subscriber                target_array_sub;       /**< Subscriber to target topic */
  ros::Subscriber                desired_pose_sub;       /**< Subscriber to Shot executer's desired pose*/
  ros::Publisher                 solved_trajectory_pub;  /**< Publisher for the solve trajectroy for others */
  ros::ServiceServer             service_for_activation; /**< service to activate the planning */
  std::map<int, ros::Subscriber> drone_pose_sub;         /**< subscribers of the drones poses <drone_id, pose_subscriber> */
  std::map<int, ros::Subscriber> drone_trajectory_sub;   /**< subscribers the solved trajectory of others <drone_id, trajectory_subscriber */
  
  // to visualize corridor
  ros::Publisher pub_path_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_corridor_polyhedrons_;
  ros::Publisher pub_corridor_ellipsoids_;

  // to test corridor
  ros::Subscriber sub_path_;
  
  // aux flags
  // std::map<int, bool> has_poses; /**< map to register if the poses of the robots have been received <drone_id, received> drone_id = 0 -> target */
  std::map<int, bool> trajectory_solved_received; /**<  flags to receive the solved trajectories for others <drone_id, received> */
  bool                is_initialized    = false;  /**< object inizialized */
  bool                activated_        = false;  /**< planning activated */
  bool                first_activation_ = true;   /**< first activation of the planning */
  bool                planning_done_    = false;  /**< planning finished */
  // timers and threads
  ros::Timer  diagnostic_timer_; /**< timer to publish diagnostic topic */

  bool target_has_pose = false; /**< has_poses[TARGET] */

  friend SolverUtils::Logger;
  SolverUtils::Logger* logger;

  std::string trajectory_frame_;

  bool         hovering_ = true;
  std::unique_ptr<NumericalSolver::ACADOSolver> solver_pt_;
  // FORCESPROsolver solver_;                /**< solver object */

  bool desired_position_reached_ = false; /**< flag to check if the last generated trajectory reach the desired point */


  /** Utility function to test the safe corridor in solver node
   */
  void referencePathCallback(const nav_msgs::PathConstPtr &msg);
  
  void sfg_test();


  /** \brief This function save the trajectory calculated by the solver **/

  void saveCalculatedTrajectory();

  /*! \brief search the closest point of the trajectory
   *   \return closest point of the trajectory
   */
  int closestPose();
  /*! \brief utility function to calculate the delay of solving
   *   \param start time to call the solver
   *   \return number of points of the calculated trajectory that are delayed
   */
  float checkRoundedTime(std::chrono::system_clock::time_point start);
  /*! \brief Utility function to erase the points that are delayed
   *   \param number_of_points that are delayed
   */
  // void deletingPoints(const int number_of_points);

  /*! \brief utility function to calculate whether last trajectory reaches the desired pose
   *    \param trajectory
   *    \param desired_pose_
   *    \return success
   */
  bool isDesiredPoseReached(const nav_msgs::Odometry &_desired_pose, const nav_msgs::Odometry &_last_pose);
  ///////////////// CALLBACKS MEMBERS ///////////////////////////////
  /*!  \brief callback for the desired pose. This function receives the pose in quaternion
   *   \param msg
   *   \TODO check yaw, not being save anymore
   */
  void desiredPoseCallback(const shot_executer::DesiredShot::ConstPtr &msg);

  /*!  \brief Callback
   *   \param msg solved trajectory of others saved in uavs_trajectory[drone_id]
   *   \param id drone_id
   */
  void uavTrajectoryCallback(const optimal_control_interface::Solver::ConstPtr &msg, int id);
  /*!  \brief receives uav's pose and save it in uavs_pose[id]. If the solved trajectory from other is not received, it initializa solved_trajectory with this
   * pose \param msg uav's pose \param id  drone_id
   */
  void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

  //////////////// UTILITY FUNCTION ////////////////////////////////
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


  /*! \brief publish the solved trajectory for others
   **/
  virtual void publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int delayed_points = 0);

  /** \brief Utility function to calculate if the trajectory calculated by the solver finishes in the desired pose
   *  \param desired_pos      This is the desired pose
   *  \param last_traj_pos    This is the last point of the calculated trajectory
   *  \return                 it will return true if the calculated trajectory reach the desired position
   */
  bool desiredPoseReached(const std::vector<double> desired_pos, const std::vector<double> last_traj_pos);
    
  /*! \brief loop to be inizialized but doing nothing
   */
  void IDLEState();
  /**! \brief Utility function to expand the position to comply with the no fly zone constraint
   *          Used for the initial guess
   */
  std::array<float, 2> expandPose(float x, float y);
  /**! \brief calculate no fly zone points. Used to make to intial guess outside the constraints
   *   \param x_center
   *   \param y_center
   *   \param radius
   *   \return nothing but the calculation is saved in no_fly_zone_points_
   */
  void calculateNoFlyZonePoints(const float x_center, const float y_center, const float radius);
  /**! \brief Calculate initial guess as straight line to the desired point or as the previous calculation
   *          Saturate velocity to not guess initial states out of the constraints
   *          accel to zero
   *          velocity cte
   *          vel cte model for path guess
   */
  void calculateInitialGuess(bool new_initial_guess = false);
};

#endif
