#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import SetBool 
from mrs_msgs.srv import Vec4Request
from mrs_msgs.srv import Vec4
from shot_executer.srv import ShootingActionRequest
from shot_executer.srv import ShootingAction
from random import randint, uniform,random
import time
from nav_msgs.msg import Odometry
import math
import numpy
from balloon_filter.srv import StartEstimationRequest
from balloon_filter.srv import StartEstimation

# target_pose = Odometry()
# action_flag = False
# drone_pose = Odometry()

# def callback_drone_pose(data):
#     global drone_pose
#     drone_pose = data

# def callback(data):
#     global target_pose
#     target_pose = data
#     if(action_flag==False):
#         target_pose = numpy.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
#         drone =  numpy.array([drone_pose.pose.pose.position.x,drone_pose.pose.pose.position.y,drone_pose.pose.pose.position.z])
#         q_camera_target = drone-target_pose
#         yaw = math.atan2(-q_camera_target[1],-q_camera_target[0])
#         srv = Vec4Request()
#         srv.goal = [drone[0],drone[1], drone[2], yaw]
#         yaw_srv(srv)



   
if __name__ == "__main__":
    
    rospy.init_node('experiment_preparation', anonymous=True)

    init_planning = SetBool()

    planning_2 = rospy.ServiceProxy('/uav2/formation_church_planning/toggle_state', SetBool)
    planning_3 = rospy.ServiceProxy('/uav3/formation_church_planning/toggle_state', SetBool)
    start_vision = rospy.ServiceProxy('/uav1/balloon_filter/start_estimation',StartEstimation)
    desired_pose = rospy.ServiceProxy('/uav1/action',ShootingAction)
    # rospy.Subscriber("/gazebo/dynamic_model/jeff_electrician/odometry", Odometry, callback)
    # rospy.Subscriber("/uav1/odometry/odom_main", Odometry, callback_drone_pose)
    yaw_srv = rospy.ServiceProxy("/uav1/control_manager/goto",Vec4)
    
    key = raw_input("press a key to start estimation")
    start_estimation_srv = StartEstimationRequest()
    start_estimation_srv.radius = 20
    start_vision(start_estimation_srv)
   # key = raw_input("press a key to start the ballon filter")
    while(1):
        key = raw_input("press a key to start the mission")

        action_flag = True

        # try:
        #     planning_2(True)
        # except:
        #     print("call planning 2 failed")

        # shooting_action = ShootingActionRequest()
        # shooting_action.shooting_action_type = ShootingActionRequest.FOLLOW
        # #relative position
        # shooting_action.rt_parameter.x = 10
        # shooting_action.rt_parameter.y = 0
        # shooting_action.rt_parameter.z = 3
        # try:
        #     desired_pose(shooting_action)
        # except:
        #     print("fail to call shooting action")

        # try:
        #     planning_2(True)
        # except:
        #     print("call planning 2 failed")

        shooting_action = ShootingActionRequest()
        shooting_action.shooting_action_type = ShootingActionRequest.ESTABLISH
        #relative position
        shooting_action.rt_parameter.x = 10
        shooting_action.rt_parameter.y = -10
        shooting_action.rt_parameter.z = 3
        try:
            desired_pose(shooting_action)
        except:
            print("fail to call shooting action")

        raw_input("next shot")
        shooting_action = ShootingActionRequest()
        shooting_action.shooting_action_type = ShootingActionRequest.ESTABLISH
        #relative position
        shooting_action.rt_parameter.x = -10
        shooting_action.rt_parameter.y = 10
        shooting_action.rt_parameter.z = 3
        try:
            desired_pose(shooting_action)
        except:
            print("fail to call shooting action")
    # while(1):

        # shooting_action = ShootingActionRequest()
        # shooting_action.shooting_action_type = ShootingActionRequest.FOLLOW
        # #relative position
        # shooting_action.rt_parameter.x = randint(-30,30)
        # shooting_action.rt_parameter.y = randint(-30,30)
        # shooting_action.rt_parameter.z = 6
        # try:
        #     desired_pose(shooting_action)
        # except:
        #     print("fail to call shooting action")
        # time.sleep(30)
    

    


    
