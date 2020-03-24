#include <shot_executer.h>

/** \brief main function of the shot executer node
*/

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "shot_executer");
    ros::NodeHandle nh;

    ShotExecuter shotExecuter(nh);

    ros::spin();
    return 0;
}
