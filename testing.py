#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from math import atan2,pi,radians,sqrt,pow

x=0.0
y=0.0
theta=0.0

def newOdom(msg):
	global x
	global y
	global theta



	x=msg.pose.pose.position.x    #Receive the x-position
	y=msg.pose.pose.position.y    #Receive the y-position

	rot_q=msg.pose.pose.orientation  #Get the Quaterion from the message
	(roll,pitch,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])  #To get the required angles from quaterion

rospy.init_node('move_tb2_test', anonymous=True)
sub=rospy.Subscriber("/marvin/diff_drive_controller/odom", Odometry, newOdom)
pub=rospy.Publisher("/marvin/diff_drive_controller/cmd_vel",Twist,queue_size=1)
r=rospy.Rate(4)

goal_distance = rospy.get_param("~goal_distance", 2.0)
goal_angle = rospy.get_param("~goal_angle", radians(90))
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians

position = Point()



for i in range(5):
	move_cmd = Twist()
	move_cmd.linear.x=0.2
	move_cmd.angular.z=0.0

	position.x=x
	position.y=y
	rotation=theta

	x_start=position.x
	y_start=position.y

	# Keep track of the distance traveled
	distance = 0
	while distance < goal_distance and not rospy.is_shutdown():
		pub.publish(move_cmd)
		r.sleep()

		# Get the current position
		position.x=x
		position.y=y
		rotation=theta
		distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

	# Stop the robot before rotating
	move_cmd = Twist()
	pub.publish(move_cmd)
	rospy.sleep(1.0)

	# Set the movement command to a rotation
	move_cmd.angular.z = 0.7

	# Track the last angle measured
	last_angle = rotation
	# Track how far we have turned
	turn_angle = 0

	while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
		# Publish the Twist message and sleep 1 cycle 
		pub.publish(move_cmd)
		r.sleep()

		# Get the current position
		position.x=x
		position.y=y
		rotation=theta

		# Compute the amount of rotation since the last loop
		delta_angle =(rotation - last_angle)

		turn_angle =turn_angle + delta_angle

		last_angle = rotation

	move_cmd = Twist()
	pub.publish(move_cmd)
	rospy.sleep(1.0)

# Stop the robot when we are done
print(i)
pub.publish(Twist())




while not rospy.is_shutdown():
	goal.x=5
	goal.y=5
	angle_to_goal=90*pi/180
	








	
