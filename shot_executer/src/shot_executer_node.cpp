#include <shot_executer_UAL.h>
#include <shot_executer_MRS.h>
/** \brief main function of the shot executer node
*/

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "shot_executer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool mrs_interface = false;
    if (ros::param::has("~mrs_interface")) {
        if (!ros::param::get("~mrs_interface", mrs_interface)) {
        ROS_ERROR("MRS interface does not have the rigth type");
        }
    } else {
        ROS_ERROR("fail to get the mrs interface param");
    }
    

    std::unique_ptr<ShotExecuter> shot_executer_interface;

    if(mrs_interface){
        shot_executer_interface = std::make_unique<ShotExecuterMRS>(nh,pnh);
        
    }else{
        shot_executer_interface = std::make_unique<ShotExecuterUAL>(nh,pnh);
    }
    ros::spin();
    return 0;
}
