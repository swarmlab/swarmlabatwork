#!/usr/bin/env python

import roslib;
roslib.load_manifest('slaw_calibration')
import rospy
from sensor_msgs.msg import LaserScan
import math
from math import sin, cos, atan2

class Calibration:
    def __init__(self):
        self.rotation_offset = rospy.get_param('~rotation_offset')
        print self.rotation_offset
        #self.min_angle = rospy.get_param('min_angle', -0.5)
        #self.max_angle = rospy.get_param('max_angle', 0.5)
        self.sub = rospy.Subscriber('base_scan', LaserScan, self.scan_cb)

        self.count = 0
        self.sum = 0

    def scan_cb(self, msg):
        angle = msg.angle_min
        d_angle = msg.angle_increment
        sum_x = 0
        sum_y = 0
        sum_xx = 0
        sum_xy = 0
        num = 0
        for r in msg.ranges:
            x = sin(angle) * r
            y = cos(angle) * r
            sum_x += x
            sum_y += y
            sum_xx += x*x
            sum_xy += x*y
            num += 1
                #print r
            angle += d_angle
        angle=atan2((-sum_x*sum_y+num*sum_xy)/(num*sum_xx-sum_x*sum_x), 1)
        self.sum += angle
        self.count += 1
        print rospy.get_name(), self.sum/self.count+self.rotation_offset, (self.sum/self.count+self.rotation_offset)* 180/math.pi 

def main():
    rospy.init_node('calibration')
    c = Calibration()
    rospy.spin()
    
if __name__ == '__main__':
    main()

