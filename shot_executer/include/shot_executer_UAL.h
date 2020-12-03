#include <shot_executer.h>

// #include <uav_abstraction_layer/TakeOff.h>
// #include <uav_abstraction_layer/GoToWaypoint.h>
// #include <uav_abstraction_layer/Land.h>

class ShotExecuterUAL : public ShotExecuter{
    public:
        ShotExecuterUAL(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);
    private:
        const std::string frame_ = "map";
        ros::Subscriber ual_pose_subscriber_;
        void ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

