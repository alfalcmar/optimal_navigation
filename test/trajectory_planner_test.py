import subprocess
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plot
import rospy
from multidrone_msgs.msg import SolvedTrajectory

#roscore_subprocess = subprocess.Popen(['roscore'])

rospy.init_node('test', anonymous=True)

#interfaces
topic_desired_pose = '/drone_1/solver/desired_pose'
topic_calculated_trajectory = '/drone_1/solver/trajectory'
trajectory_received = False
zero_time = rospy.Time.now()
def callback(data):
    global trajectory_received
    trajectory_received = True
    print("Interface test passed: trajectory received")
    x = []
    y = []
    z = []
    for point in data.positions:
        x.append(point.x)
        y.append(point.y)
        z.append(point.z)
    plot.plot(x,y)
    plot.show()
    print(x)

#interfaces
topic_desired_pose = '/drone_1/solver/desired_pose'
topic_calculated_trajectory = '/drone_1/solver/trajectory'

sub = rospy.Subscriber(topic_calculated_trajectory,SolvedTrajectory,callback)

#test interfaces

process = subprocess.Popen(['roslaunch', 'optimal_control_interface', 'opt_control_node.launch'])

pub = rospy.Publisher(topic_desired_pose,PoseStamped)

pub_fake_pose = rospy.Publisher('/drone_1/ual/pose',PoseStamped)
pub_desired_pose = rospy.Publisher('/drone_1/solver/desired_pose',PoseStamped)
rate = rospy.Rate(1) # hz
while not rospy.is_shutdown() or test_finished:
    rate.sleep()
    fake_pose = PoseStamped()
    fake_pose.pose.position.x = -3
    fake_pose.pose.position.y = 0
    fake_pose.pose.position.z = 1
    
    fake_desired_pose = PoseStamped()
    fake_desired_pose.pose.position.x = 8
    fake_desired_pose.pose.position.y = 0
    fake_desired_pose.pose.position.z = 1
    pub_fake_pose.publish(fake_pose)
    pub_desired_pose.publish(fake_desired_pose)

    time_now = rospy.Time.now()-zero_time
    if(not trajectory_received and (time_now.secs>10)):
        break
print("Test failed, trajectory don't received")
process = subprocess.Popen(['rosnode', 'kill', '-a'])

