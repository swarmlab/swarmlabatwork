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


odom = "/odom"
base_frame = "/base_link"

class OdomMovePidMonitor:
    x = y = theta = 0



    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)
        self.stopped = True
        self.eps_x = 0.01
        self.eps_y = 0.01
        self.eps_theta = 0.01
        self.side = True
        self.odomTargetPose = Pose2D()
        rospy.Subscriber("/odom", Odometry, self.cb_odom)

        self.pid_pub = rospy.Publisher('/scan_registration/tarDir', Pose2D)
        #change pid params?
        self.stopPid = rospy.ServiceProxy('scan_registration/switchOffPID', switchOff)
        self.action_server = actionlib.simple_action_server.SimpleActionServer('odom_fine_adjust', OdomFineAdjustAction, self.execute_cb, False)
        self.action_server.start()


    def execute_cb(self, goal):
        if abs(goal.target.x) > 0:
            self.side = True
        elif abs(goal.target.y) > 0:
            self.side = False
        self.set_target_with_offset(goal.target.x, goal.target.y, goal.target.theta)
        self.eps_x = goal.threshold.x
        self.eps_y = goal.threshold.y
        self.eps_theta = goal.threshold.theta
        self.stopped = False

        rate_hz = 10
        rate = rospy.Rate(rate_hz)
        counter = 0
        
        result = OdomFineAdjustResult()
        while not self.is_done() and counter < goal.duration*rate_hz:
            counter += 1
            result.success = False
            result.result.x = self.x
            result.result.y = self.y
            result.result.theta = self.theta
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                self.action_server.set_aborted(result)
                self.stop()
                return
            rate.sleep()
        self.stop()
        print counter
        result.success = True
        self.action_server.set_succeeded(result)



    def is_done(self):
        if abs(self.x)>self.eps_x or  abs(self.y)>self.eps_y or abs(self.theta)>self.eps_theta:
            return False
        return True
    
    
    def stop(self):
        self.stopped = True
       # self.set_target_with_offset(0,0,0)
        self.pub_pid_target()
        req = switchOffRequest()
        req.pause = self.stopped
        #rospy.wait_for_service('/scan_registration/switchOffRegistration')
        try:
            resp = self.stopPid(req)
            print "pid stopped ", resp.success
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))


        
    def cb_odom(self,msg):
        if self.stopped:
            return
        
        #self.set_target_with_offset(0.0, -0.2, 0)
        
        self.calc_diff(self.odom_to_2D_Pose(msg))
        self.pub_pid_target()

        
    def odom_to_2D_Pose(self, odom_msg):
        pose = Pose2D()
        pose.x = odom_msg.pose.pose.position.x
        pose.y = odom_msg.pose.pose.position.y
        quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w]

        r,p, pose.theta = tf.transformations.euler_from_quaternion(quat)
        return pose


    def transformPose(self, pose_in):
       
        if self.tfListen.frameExists(base_frame) and self.tfListen.frameExists(odom):

            time = self.tfListen.getLatestCommonTime(odom, base_frame)
            pose_in.header.stamp = time
            pose = self.tfListen.transformPose(odom, pose_in)
            return pose
        return None
            
    def set_target_with_offset(self, x, y, theta): 
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.orientation.w = 1.0
        
        tar = self.transformPose(pose_stamped)
        
        self.odomTargetPose.x = tar.pose.position.x
        self.odomTargetPose.y = tar.pose.position.y

        quat = [tar.pose.orientation.x, tar.pose.orientation.y, tar.pose.orientation.z,tar.pose.orientation.w]

        r,p, theta = tf.transformations.euler_from_quaternion(quat)
        self.odomTargetPose.theta = theta

        self.x = x
        self.y = y
        self.theta = theta

        print self.odomTargetPose
        
    def calc_diff(self, pose):
        self.x = self.odomTargetPose.x - pose.x
        self.y = self.odomTargetPose.y - pose.y
        self.theta = self.odomTargetPose.theta - pose.theta


    def pub_pid_target(self):
        pose = Pose2D()
        #pose.x = self.x / 2.
#todo
        if self.side == True:
            pose.y = 0

            if self.x > 0.025:
                pose.x = 0.05
            elif self.x < -0.025:
                pose.x = -0.05
            else:
                pose.x = 0
        else:
            pose.x = 0
            if self.y > 0.025:
                pose.y = 0.05
            elif self.y < -0.025:
                pose.y = -0.05
        pose.theta = 0

        self.pid_pub.publish(pose)

    def loop(self):
        rate = rospy.Rate(5)
        print "enter commands"
        while not rospy.is_shutdown():
            i,o,e = select.select([sys.stdin],[],[],0.0001)
            for s in i:
                if s == sys.stdin:
                    input_cmd = sys.stdin.readline()
                    self.processKey(input_cmd)
            rate.sleep()   


    def processKey(self, key):
        if key == 'forward-5\n':
            print "fw 5"
            tuck_arm_client = actionlib.SimpleActionClient('odom_fine_adjust', OdomFineAdjustAction)
            goal = OdomFineAdjustGoal()
            goal.target.x = 0.05
            goal.target.y = 0
            goal.threshold.x = 0.05
            goal.threshold.y = 0.05
            goal.threshold.theta = 1
            goal.duration = 10;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"

        if key == 'back-20\n':
            print "back 20"
            tuck_arm_client = actionlib.SimpleActionClient('odom_fine_adjust', OdomFineAdjustAction)
            goal = OdomFineAdjustGoal()
            goal.target.x = -0.2
            goal.target.y = 0
            goal.threshold.x = 0.05
            goal.threshold.y = 0.05
            goal.threshold.theta = 1
            goal.duration = 10;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"

        if key == 'forward-20\n':
            print "fw 20"
            tuck_arm_client = actionlib.SimpleActionClient('odom_fine_adjust', OdomFineAdjustAction)
            goal = OdomFineAdjustGoal()
            goal.target.x = 0.2
            goal.target.y = 0
            goal.threshold.x = 0.05
            goal.threshold.y = 0.05
            goal.threshold.theta = 1
            goal.duration = 10;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"

            
        if key == 'right-5\n':
            print "right 5"
            tuck_arm_client = actionlib.SimpleActionClient('odom_fine_adjust', OdomFineAdjustAction)
            goal = OdomFineAdjustGoal()
            goal.target.x = 0
            goal.target.y = -0.05
            goal.threshold.x = 0.01
            goal.threshold.y = 0.01
            goal.threshold.theta = 0.01
            goal.duration = 10;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"

        if key == 'left-5\n':
            print "left 5"
            tuck_arm_client = actionlib.SimpleActionClient('odom_fine_adjust', OdomFineAdjustAction)
            goal = OdomFineAdjustGoal()
            goal.target.x = 0
            goal.target.y = 0.05
            goal.threshold.x = 0.01
            goal.threshold.y = 0.01
            goal.threshold.theta = 0.01
            goal.duration = 10;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"

        
if __name__ == '__main__':
    rospy.init_node('odom_monitor', anonymous = False)
    mon = OdomMovePidMonitor()
    mon.loop()
    #rospy.spin()
