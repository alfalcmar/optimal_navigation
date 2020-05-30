#!/usr/bin/env python
import rospy
from shot_executer.msg import DesiredShot
from nav_msgs.msg import Odometry

desired_odometry = Odometry()

def callback(data):
    global desired_odometry
    print("pose received")
    desired_odometry = data.desired_odometry

if __name__== '__main__':
    rospy.init_node('visualizer', anonymous=True)
    rospy.Subscriber("/uav1/desired_pose", DesiredShot, callback)
    pub = rospy.Publisher('desired_pose',Odometry,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        desired_odometry.header.frame_id ='uav1/gps_origin'
        pub.publish(desired_odometry)

        
        