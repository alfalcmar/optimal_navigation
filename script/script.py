#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import SetBool 
from mrs_msgs.srv import Vec4
from shot_executer.srv import ShootingActionRequest
from shot_executer.srv import ShootingAction
from random import randint, uniform,random
import time


   
if __name__ == "__main__":
    
    init_planning = SetBool()

    planning_2 = rospy.ServiceProxy('/uav2/formation_church_planning/toggle_state', SetBool)
    planning_3 = rospy.ServiceProxy('/uav3/formation_church_planning/toggle_state', SetBool)
    desired_pose = rospy.ServiceProxy('/uav1/action',ShootingAction)



    try:
        planning_2(True)
    except:
        print("call planning 2 failed")

    shooting_action = ShootingActionRequest()
    shooting_action.shooting_action_type = ShootingActionRequest.FOLLOW
    #relative position
    shooting_action.rt_parameter.x = 10
    shooting_action.rt_parameter.y = 0#randint(-30,30)
    shooting_action.rt_parameter.z = 3
    try:
        desired_pose(shooting_action)
    except:
        print("fail to call shooting action")
    # while(1):

    #     shooting_action = ShootingActionRequest()
    #     shooting_action.shooting_action_type = ShootingActionRequest.FOLLOW
    #     #relative position
    #     shooting_action.rt_parameter.x = randint(-30,30)
    #     shooting_action.rt_parameter.y = randint(-30,30)
    #     shooting_action.rt_parameter.z = 6
    #     try:
    #         desired_pose(shooting_action)
    #     except:
    #         print("fail to call shooting action")
    #     time.sleep(30)
    

    


    
