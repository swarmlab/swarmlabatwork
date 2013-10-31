#!/usr/bin/env python

import roslib;
roslib.load_manifest('mitro_calibration')
import rospy
import math

from geometry_msgs.msg import Twist
from mitro_base_controller.msg import JointStates
from nav_msgs.msg import Odometry

import sys

global left, right, new, speed_linear, twist, timer
left = 0
right = 0
new = False
speed_linear = 0


def js_cb(msg):
	global left, right, new

	new = True
	left = msg.position_left
	right = msg.position_right

def odom_cb(msg):
	#print msg.twist.twist.linear.x
	if msg.twist.twist.linear.x > 0.499:
	#if msg.twist.twist.angular.z > 1:
		twist.angular.z = 0
		twist.linear.x = 0
		print (msg.header.stamp - timer).to_sec()


if __name__ == '__main__':
	rospy.init_node('DriveStraight')
	rospy.Subscriber('joint_states', JointStates, js_cb)
	rospy.Subscriber('odom', Odometry, odom_cb)
	pub = rospy.Publisher('cmd_twist', Twist)
	global twist, timer
	twist = Twist()
	twist.linear.x = 0
	cnt = 0

	new = False
	state = 0

	timer = rospy.Time.now()

	while not rospy.is_shutdown():
	

		
		if state == 0:
			if new:
				print left, right
				start_l = left
				start_r = right
				state = 1
				twist.linear.x = 0.5
				#twist.angular.z = 1.0

		elif state == 1:
			cnt += 1
			if cnt == 160:
				state = 2
				twist.linear.x = 0.0
				cnt = 0
		elif state == 2:
			cnt += 1
			if cnt == 20:
				print left, right
				print left - start_l, right - start_r
		pub.publish(twist)
		rospy.sleep(0.05)
		

