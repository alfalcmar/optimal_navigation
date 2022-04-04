#include <backendSolver.h>
#include <formation_church_planning/Trajectory.h>
#include <formation_church_planning/Point.h>
#include <formation_church_planning/Diagnostic.h>
#include <formation_church_planning/Status.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/String.h>


class backendSolverMRS : public backendSolver {
public:
  backendSolverMRS(ros::NodeHandle &_pnh, ros::NodeHandle &_nh, const int time_horizon);

private:
  std::unique_ptr<mrs_lib::Transformer> transformer_;

  ros::Subscriber uav_odometry_sub; /**< Subscriber to UAV's odometry*/
  ros::Publisher  diagnostics_pub;
  ros::Publisher  solved_trajectory_MRS_pub;
  ros::Publisher  target_odometry_pub;
  ros::Publisher  mrs_trajectory_tracker_pub;
  ros::Publisher  mrs_status_pub;

  /*! \brief Callback function to the target topic. This function save the pose reveived for the first in has_poses[0] and upload target_pose_ member
   *  \param
   * */
  void targetCallbackMRS(const nav_msgs::Odometry::ConstPtr &_msg);
  /*! \brief Service callback to activate the planning
   *   \param req req.data = true -> activate planning, req.data = false -> deactivate planning
   *   \param res res.message = "message"
   *   \return planning_activated
   **/
  bool activationServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void uavCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void publishSolvedTrajectory(const std::vector<double> &yaw, const std::vector<double> &pitch, const int delayed_points = 0);
  void diagTimer(const ros::TimerEvent &event);

  void publishTargetOdometry();
  void publishState(const bool state);
  void uavPoseCallback(const nav_msgs::Odometry::ConstPtr &msg, int id);

};
