#include <shot_executer.h>

#ifdef UAL
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>
#endif

#ifdef UAL
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

#endif