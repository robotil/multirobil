#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
 
def callback(data):
	pub_tilt.publish(data.axes[1])
	pub_pan.publish(data.axes[0])
	twist = Twist()
	twist.linear.x = data.axes[3]
	twist.angular.z = 3*data.axes[2]  
	pub.publish(twist)
 
def joy_teleop():
	rospy.init_node('Joy_teleop')
	rospy.Subscriber("joy", Joy, callback)
	global pub
	global pub_pan
	global pub_tilt
	pub_tilt = rospy.Publisher('/joint2_position_controller/command', Float64)
	pub_pan = rospy.Publisher('/joint1_position_controller/command', Float64)
	pub = rospy.Publisher('/cmd_vel', Twist)  
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		r.sleep()
 
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	joy_teleop()
