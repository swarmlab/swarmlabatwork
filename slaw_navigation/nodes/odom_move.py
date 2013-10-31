#!/usr/bin/env python

import roslib
roslib.load_manifest("slaw_navigation")

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
from math import pi
from slaw_navigation.msg import *
from slaw_navigation.srv import *

import sys, select
import tf


# Node that sends twist until odom dist is greater than x
RATE = 20 # hz

class OdomMove:
    dist = None
    first_pos = None
    max_dist = None
    is_done = False
    
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)
        self.pub = rospy.Publisher("/cmd_vel", Twist)
        self.start = False
        rospy.Subscriber("/odom", Odometry, self.cb_odom)
        self.action_server = actionlib.simple_action_server.SimpleActionServer('odom_move', OdomFineAdjustAction, self.execute_cb, False)
        self.action_server.start()        

        
    def execute_cb(self, goal):

        rate = rospy.Rate(RATE)
        twist_msg = Twist()
        twist_msg.linear.x = goal.target.x
        twist_msg.linear.y = goal.target.y
        
        self.max_dist = goal.max_dist
        self.is_done = False
        counter = 0
        self.first_pos = None
        self.start = True
        while counter * 1.0/RATE < goal.duration: #not self.is_done and counter * 1/RATE < goal.duration:
            if self.action_server.is_preempt_requested():
                #stop
                twist_msg = Twist()
                self.pub.publish(twist_msg)
                result = OdomFineAdjustResult()
                result.success = False
                self.action_server.set_preempted()
                self.action_server.set_aborted(result)
                self.start = False
                return
            self.pub.publish(twist_msg)
            counter += 1 
            rate.sleep()            

        # done & stop
        twist_msg = Twist()
        self.pub.publish(twist_msg)
        rospy.loginfo("done")
        result = OdomFineAdjustResult()
        result.success = True
        self.action_server.set_succeeded(result)
        self.start = False
        
    def cb_odom(self,msg):
        if not self.start:
            return
        if self.first_pos is None:
            self.first_pos = msg.pose.pose.position
        else:
            # is done ?
            current_pos = msg.pose.pose.position
            #print  pow(self.first_pos.x - current_pos.x,2)  + pow(self.first_pos.y - current_pos.y,2) 
            if pow(self.first_pos.x - current_pos.x,2)  + pow(self.first_pos.y - current_pos.y,2) > pow(self.max_dist,2):
                self.is_done = True

           
if __name__ == '__main__':
    rospy.init_node('odom_monitor', anonymous = False)
    
    mon = OdomMove()
    rospy.spin()
    #client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
    #client.wait_for_server()
    #goal = OdomFineAdjustGoal()
    #goal.target.x = 0.0
    #goal.target.y = 0.05
    #goal.max_dist = 0.2
    #goal.duration = 15
    #client.send_goal_and_wait(goal)
