#include<shot_executer_UAL.h>

ShotExecuterUAL::ShotExecuterUAL(ros::NodeHandle &_nh, ros::NodeHandle &_pnh) : ShotExecuter::ShotExecuter(_nh,_pnh,"map"){

    ual_pose_subscriber_ = _nh.subscribe("ual/pose",1,&ShotExecuterUAL::ualPoseCallback,this);
}
void ShotExecuterUAL::ualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pose_.pose.pose = msg->pose;
}
