#include<backendSolver.h>


#ifdef UAL
class backendSolverUAL : backendSolver {
public:
  backendSolverUAL(ros::NodeHandle &_pnh, ros::NodeHandle &_nh); /**< UAL backend constructor*/
private:
  ros::Subscriber              uav_state_sub; /**< Subscriber to UAL's state*/
  ros::Subscriber              sub_velocity;  /**< Subscriber to UAL's velocity*/
  uav_abstraction_layer::State ual_state_;    /**< ual state */

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
#endif
