#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import SetBool 
from uav_abstraction_layer.srv import TakeOffRequest
from uav_abstraction_layer.srv import TakeOff
from uav_abstraction_layer.srv import GoToWaypoint
from shot_executer.srv import ShootingActionRequest
from shot_executer.srv import ShootingAction
from random import randint, uniform,random
import time
from nav_msgs.msg import Odometry
import math
import numpy
import os
import threading

target_pose = Odometry()
action_flag = False
drone_pose = Odometry()

def callback_drone_pose(data):
    global drone_pose
    drone_pose = data

def callback(data):
    global target_pose
    target_pose = data
    if(action_flag==False):
        target_pose = numpy.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        drone =  numpy.array([drone_pose.pose.pose.position.x,drone_pose.pose.pose.position.y,drone_pose.pose.pose.position.z])
        q_camera_target = drone-target_pose
        yaw = math.atan2(-q_camera_target[1],-q_camera_target[0])
        srv = Vec4Request()
        srv.goal = [drone[0],drone[1], drone[2], yaw]
        yaw_srv(srv)

def thread_function(name):
    target_odom = Odometry()
    while not rospy.is_shutdown():
        target_odom.pose.pose.position.x = 20
        target_odom.pose.pose.position.y = 0
        target_odom.pose.pose.position.z = 0
        target_fake_pub.publish(target_odom)
        time.sleep(1)


   
if __name__ == "__main__":
    
    rospy.init_node('experiment', anonymous=True)
    uav_id = "drone_1"
    ns ="/drone_1"
    take_off_service = rospy.ServiceProxy(ns+"/ual/take_off", TakeOff)
    go_to_waypoint_service = rospy.ServiceProxy(ns+'', SetBool)

    go_to_waypoint_url = "/drone_1/ual/go_to_waypoint"
    rospy.wait_for_service(go_to_waypoint_url)
    
    go_to_waypoint = rospy.ServiceProxy(go_to_waypoint_url, GoToWaypoint)
    target_fake_pub = rospy.Publisher('/target_fake', Odometry, queue_size=10)
    desired_pose = rospy.ServiceProxy('/'+uav_id+'/action',ShootingAction)
    
    mythread = threading.Thread(target=thread_function, args=(1,))
    mythread.start()

    try:
        take_off = TakeOffRequest()
        take_off.height = 2
        take_off.blocking = True
        take_off_service(take_off)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    raw_input("Establish -30 0 2")

    action_flag = True

    shooting_action = ShootingActionRequest()
    shooting_action.shooting_action_type = ShootingActionRequest.ESTABLISH
    #relative position
    shooting_action.rt_parameter.x = 10.0
    shooting_action.rt_parameter.y = -25
    shooting_action.rt_parameter.z = 2
    try:
        desired_pose(shooting_action)
    except:
        print("fail to call shooting action")

    raw_input("Lateral")
    shooting_action = ShootingActionRequest()
    shooting_action.shooting_action_type = ShootingActionRequest.FOLLOW
    #relative position
    shooting_action.rt_parameter.x = 0
    shooting_action.rt_parameter.y = -10
    shooting_action.rt_parameter.z = 2
    try:
        desired_pose(shooting_action)
    except:
        print("fail to call shooting action")


    


    
