#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from math import atan2,pi,pow,sqrt,radians
import tf

roll=pitch=theta=0.0

goal=90*pi/180
kp=0.9

def newOdom(msg):
	global x
	global y
	global theta



	x=msg.pose.pose.position.x    #Receive the x-position
	y=msg.pose.pose.position.y    #Receive the y-position

	rot_q=msg.pose.pose.orientation  #Get the Quaterion from the message
	(roll,pitch,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])  #To get the required angles from quaterion


rospy.init_node('move_tb2_test', anonymous=True) #Initiliaze the node
sub=rospy.Subscriber("/marvin/diff_drive_controller/odom", Odometry, newOdom) #Subscribe to to the topic Odometry
pub=rospy.Publisher("/marvin/diff_drive_controller/cmd_vel",Twist,queue_size=1) #Publish to the topic to cmd_vel of marvin

r=rospy.Rate(10)  #Define the rate ( rate allows your loops to run at (nearly) the exact rate (in Hz))

command =Twist() #Store the twist data
print("Theta", theta)
error=goal-theta
print("Current Error", error)
while not rospy.is_shutdown() and error>0.0001 :
	
	error=goal-theta
	print("Error is :",error)
	command.angular.z=kp*error
	pub.publish(command)
	r.sleep()


