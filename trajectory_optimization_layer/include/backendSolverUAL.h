#ifndef BACKENDSOLVERUAL_H
#define BACKENDSOLVERUAL_H
#include<backendSolver.h>

class backendSolverUAL : public backendSolver {
public:
  backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh, const int time_horizon);
private:
  ros::Subscriber              uav_state_sub; /**< Subscriber to UAL's state*/
  ros::Subscriber              sub_velocity;  /**< Subscriber to UAL's velocity*/

  /*!  \brief callback that save the ual velocity
   *   \param msg
   */
  void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg); /**< Callback for UAL's state topic */
  /*!  \brief target pose topic callback. This function save the first time received and the msg into target_pose_
   *   \param msg topic msg
   */
  void targetPoseCallbackGRVC(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif