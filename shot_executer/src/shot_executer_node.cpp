#include <shot_executer_MRS.h>

/** \brief main function of the shot executer node
*/

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "shot_executer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ShotExecuterMRS shotExecuterMRS(nh,pnh);

    ros::spin();
    return 0;
}
