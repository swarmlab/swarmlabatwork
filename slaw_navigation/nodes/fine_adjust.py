#!/usr/bin/env python

import roslib
roslib.load_manifest("slaw_navigation")

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import pi
from slaw_navigation.msg import *


LEFT = 1
RIGHT = -1

LASER_2_WHEEL_F = 0.17
LASER_2_WHEEL_R = 0.17
LASER_OFF_F = 0.03
LASER_OFF_R = 0.03

MIN_DIST_X = 0.10
MIN_DIST_Y = 0.02
MAX_DIFF = 0.2


MIN_DIFF = 0.005

class FineAdjust:

    fleft = 0
    fright = 0
    rleft = 0
    rright = 0

    front = 0
    rear = 0
    

    def __init__(self):
        rospy.Subscriber("/base_scan_front", LaserScan, self.cb_front)
        rospy.Subscriber("/base_scan_rear", LaserScan, self.cb_rear)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist)

        self.action_server = actionlib.simple_action_server.SimpleActionServer('fine_adjust', FineAdjustAction, self.executeCB)
        self.initial_front = None

    def executeCB(self, goal):
        global MIN_DIFF
        if self.initial_front is None: # just set once
            self.initial_front = self.front
        max_iterations = goal.iterations
        distance = goal.distance
        offset = goal.offset
        use_offset = goal.use_offset
        result = FineAdjustResult()
        if goal.threshold > 0:
            MIN_DIFF = goal.threshold
        for i in xrange(max_iterations):
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                result.result = False
                break
            result.result = self.control(distance, offset, use_offset)
           
            if result.result:
                self.action_server.set_succeeded(result)
                rospy.loginfo("done")
                return
        rospy.loginfo("return")
        self.action_server.set_aborted(result)
        
    def cb_front(self, msg):
        #    rospy.loginfo("got front laser msg")
        self.fleft = self.get_dist(pi/2, msg) - LASER_2_WHEEL_F
        self.fright = self.get_dist(-pi/2, msg) - LASER_2_WHEEL_F
        self.front = self.get_dist(0, msg) - LASER_OFF_F
        #rospy.loginfo("left: %f\t\tright: %f\t\tfront: %f"%(self.fleft, self.fright, self.front))
    
    def cb_rear(self, msg):
        #    rospy.loginfo("got rear laser msg")
        self.rleft = self.get_dist(-pi/2, msg) - LASER_2_WHEEL_R
        self.rright = self.get_dist(pi/2, msg) - LASER_2_WHEEL_R
        self.rear = self.get_dist(0, msg) - LASER_OFF_R
        #rospy.loginfo("\t\t\t\t\t\t\t\t\tleft: %f\t\tright: %f\t\trear: %f"%(self.rleft, self.rright, self.rear))
    
    def get_dist(self, angle, msg, n=2):
        # gets distance closest to angle
        # average over (at most) n left and right of target angle
        if angle < msg.angle_min or angle > msg.angle_max:
            rospy.logerr("can't get distance, angle %f outside range(%f,%f)"%(angle, msg.angle_min, msg.angle_max))
            return None
        i = int((angle-msg.angle_min) / msg.angle_increment)

        avg = 0
        cnt = 0
        for j in xrange(i-n,i+n+1):
            if j < 0 or j >= len(msg.ranges):
                continue
            if msg.ranges[j] >= msg.range_min and msg.ranges[j] <= msg.range_max:
                avg += msg.ranges[j]
                cnt += 1
        if cnt > 0:
            return avg*1.0/cnt
        else:
            return msg.range_max

    def send_twist(self, turn_speed, side_speed, x, sleep=0.1):
        msg = Twist()
        # msg.linear.x = 0
        # msg.linear.y = 0
        # msg.linear.z = 0

        # normalize for safety
        #direction = direction / abs(direction)
        
        msg.angular.z = turn_speed
        msg.linear.y = side_speed
        msg.linear.x = x
        #rospy.loginfo("send twist: %s"%str(msg))
        self.pub_twist.publish(msg)
        
        rospy.sleep(sleep)    
    
        msg_stop = Twist()
        self.pub_twist.publish(msg_stop)

        rospy.sleep(sleep)    
    
    def send_rotation(self, direction, speed=0.2, sleep=0.1):
        msg = Twist()
        # msg.linear.x = 0
        # msg.linear.y = 0
        # msg.linear.z = 0

        # normalize for safety
        direction = direction / abs(direction)
        
        msg.angular.z = direction * speed
        rospy.loginfo("send twist: %s"%str(msg))
        self.pub_twist.publish(msg)
        
        rospy.sleep(sleep)    
    
        msg_stop = Twist()
        self.pub_twist.publish(msg_stop)

        rospy.sleep(sleep)    

    def send_side(self, direction, speed=0.1, sleep=0.1):
        msg = Twist()
        # msg.linear.x = 0
        # msg.linear.y = 0
        # msg.linear.z = 0

        # normalize for safety
        direction = direction / abs(direction)
        
        msg.linear.y = direction * speed
        rospy.loginfo("send twist: %s"%str(msg))
        self.pub_twist.publish(msg)
        
        rospy.sleep(sleep)    
    
        msg_stop = Twist()
        self.pub_twist.publish(msg_stop)

        rospy.sleep(sleep)    

    counter = 0

    def get_side(self, target):
        # case for left
        dist = (self.fleft + self.rleft)/2
        dif = dist - target
        
        print dif
        
        max_dif = 0.055
        min_dif = 0.005

        max_speed = 0.15
        min_speed = 0.05

        speed = (dif-min_dif)/(max_dif-min_dif)*(max_speed-min_speed)+min_speed
        speed = max(speed, min_speed)        
        speed = min(speed, max_speed)

        # print speed
        if abs(dif) > MIN_DIFF:
            if dif < 0:
                return speed * RIGHT
#                self.send_side(RIGHT, speed = speed)
#                rospy.logerr("RIGHT!")
            else:
                return speed * LEFT
#                self.send_side(LEFT, speed = speed)
#                rospy.logerr("LEFT!")
#            self.counter = 0
#            return False
        else:
            return 0
#            self.counter += 1
#            return self.counter > 4

    
    def get_rot(self):
        # case for left
        dif = self.fleft - self.rleft

        print dif
        
        max_dif = 0.055
        min_dif = 0.005

        max_speed = 0.35
        min_speed = 0.2

        speed = (dif-min_dif)/(max_dif-min_dif)*(max_speed-min_speed)+min_speed
        speed = max(speed, min_speed)        
        speed = min(speed, max_speed)
        # print speed
        if abs(dif) > MIN_DIFF:
            if dif < 0:
                return speed * RIGHT
#                self.send_rotation(RIGHT, speed = speed)
#                rospy.logerr("RIGHT!")
            else:
                return speed * LEFT
#                self.send_rotation(LEFT, speed = speed)
#                rospy.logerr("LEFT!")
#            self.counter = 0
#            return False
        else:
            return 0
#            self.counter += 1
#            return self.counter > 4


    counter = 0
        
    def control(self, side_target, front_offset, use_offset):
        rot = self.get_rot()
        side = self.get_side(side_target)

        x = 0

        if use_offset:
        # front laser bumber
            front_goal = self.initial_front - front_offset
            if abs(self.front - front_goal) > 0.03:
                if self.front - front_goal > 0:
                # go forward
                    x = 0.1
                else:
                    x = -0.1

                if self.front < MIN_DIST_X:
                    x = min(x, 0)
                if self.rear < MIN_DIST_X:
                    x = max(x, 0)
                
        # check laser bumper
        if self.fleft < MIN_DIST_Y or self.rright < MIN_DIST_Y:
            # don't rotate left
            rot = min(rot, 0)
        if self.rleft < MIN_DIST_Y or self.fright < MIN_DIST_Y:
            # don't rotate right
            rot = max(rot, 0)

        if self.fleft < MIN_DIST_Y or self.rleft < MIN_DIST_Y:
            # don't go left
            side = min(side, 0)
        if self.fright < MIN_DIST_Y or self.rright < MIN_DIST_Y:
            # don't go right
            side = max(side, 0)

        if abs(self.fleft - self.rleft) > MAX_DIFF: # TODO only works for left adjust
            rot = 0

        self.send_twist(rot, side, x)
        if rot == 0 and side == 0:
            if not use_offset or x == 0:
                self.counter += 1
                return self.counter > 4
        else:
            self.counter = 0
            return False
            
        
if __name__ == "__main__":
    rospy.init_node("fine_adjust")
    adjust = FineAdjust()

    r = rospy.Rate(10)

    rospy.sleep(1)
    #while not adjust.control(0.03) and not rospy.is_shutdown():
    #    r.sleep() 
    rospy.spin()
    # while not adjust.control(0.10) and not rospy.is_shutdown():
    #     r.sleep()

    # while not adjust.control(0.03) and not rospy.is_shutdown():
    #     r.sleep() 


        #    while not adjust.side_control(0.30) and not rospy.is_shutdown():
#        r.sleep()
