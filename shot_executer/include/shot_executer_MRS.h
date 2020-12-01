#include <shot_executer.h>

/**
 * Class that interfaces with MRS system
 * Input: inputs inherited from the base class
 * Output: outputs inherited from the base class plus publishing the pitch command needed to point the camera to the target
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
        void uavCallback(const nav_msgs::Odometry::ConstPtr &msg);
        /*! \brief function that publishes the needed pitch to point the camera to the target
         */
        void publishCameraCommand();
};