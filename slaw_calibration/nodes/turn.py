#!/usr/bin/env python

import roslib;
roslib.load_manifest('mitro_calibration')

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	rospy.init_node('turn_node')
	pub = rospy.Publisher('cmd_twist', Twist)

	twist = Twist()
	twist.angular.z = 1.5

	time = rospy.Time.now()

	while not rospy.is_shutdown():
		pub.publish(twist)

		if (rospy.Time.now() - time).to_sec() > 10.0:
			twist.angular.z = - twist.angular.z
			time = rospy.Time.now()

		rospy.sleep(0.05)
		
