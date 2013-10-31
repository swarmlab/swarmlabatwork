#!/usr/bin/env python       

import roslib
roslib.load_manifest('slaw_registration')

import rospy
import actionlib

from slaw_registration.srv import *
from slaw_navigation.srv import *
from slaw_registration.msg import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
#from copy import deepcopy

import sys, select, termios, tty, signal

class fakeMonitor(object):


    def __init__(self):
        print "starting fake monitor"

        self.saveFront = False
        self.saveRear = False
        self.saveFrontTwo = False
        self.saveRearTwo = False
        self.frontScan = LaserScan()
        self.rearScan = LaserScan()
        self.frontScanTwo = LaserScan()
        self.rearScanTwo = LaserScan()
        self.pause = False
        self.dX = 0
        self.dY = 0
        self.dTheta = 0
                
        rospy.init_node('fakeMonitor', anonymous = False)
        rospy.Subscriber("/base_scan_front", LaserScan, self.front_cb)
        rospy.Subscriber("/base_scan_rear", LaserScan, self.rear_cb)
        rospy.Subscriber("/scan_registration/tarDir", Pose2D, self.tarDirCb)
        
        rospy.sleep(0.001)

        self.action_server = actionlib.simple_action_server.SimpleActionServer('registration_fine_adjust', RegistrationFineAdjustAction, self.executeCB, False)
        self.action_server.start()

    def tarDirCb(self, pose):
        self.dX = pose.x
        self.dY = pose.y
        self.dTheta = pose.theta

    def checkDistance(self, x, y, yaw):
        if abs(self.dX) < x and abs(self.dY) < y and abs(self.dTheta) < yaw:
            return True
        else:
            return False
        
    def executeCB(self, goal):
        self.updateTar(goal.front, goal.rear)
        max_iterations = goal.duration

        result = RegistrationFineAdjustResult()
        
        rate = rospy.Rate(1)
        for i in xrange(max_iterations):
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                result.result = False
                break

            result.result = self.checkDistance(goal.x_thresh, goal.y_thresh, goal.theta_thresh)
            result.x_result = self.dX
            result.y_result = self.dY
            result.theta_result = self.dTheta
            
            if result.result:
                self.action_server.set_succeeded(result)
                self.updatePause()
                return
            rate.sleep()
        self.updatePause()
        self.action_server.set_aborted(result)


    def front_cb(self, data):
        if self.saveFront:
            print "saved front"
            self.frontScan = data
            print self.frontScan.header.stamp
            self.saveFront = False;
        if self.saveFrontTwo:
            print "saved front two"
            self.frontScanTwo = data
            print self.frontScanTwo.header.stamp
            self.saveFrontTwo = False;

            
    def rear_cb(self, data):
        if self.saveRear:
            print "saved back"
            self.rearScan = data
            self.saveRear = False
        if self.saveRear:
            print "saved back two"
            self.rearScanTwo = data
            self.saveRearTwo = False

            
    def updateTar(self, front, back):
        success = False
        counter = 9
        while not success and counter<10:
            front.header.stamp = rospy.Time.now()
            back.header.stamp = rospy.Time.now()
            success = self.sendTar(front,back)
            counter+=1
            rospy.sleep(0.5)
        if not success:
            print "sendTar failed"
        else:
            self.pause = False

    def sendTar(self, front, back):
        rospy.wait_for_service('/scan_registration/setTar')
        try:
            set_tar = rospy.ServiceProxy('/scan_registration/setTar', setRegistrationTar)
            print front.header.stamp
            response = set_tar(front, back)
            return response.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    def updatePause(self):
        success = False
        counter = 9
        while not success and counter<10:
            success = self.sendPause()
            counter+=1
            rospy.sleep(0.5)
        if not success:
            print "sendPause failed"

            
    def sendPause(self):
        rospy.wait_for_service('/scan_registration/switchOffRegistration')
        try:
            switch_off = rospy.ServiceProxy('/scan_registration/switchOffRegistration', switchOff)
            self.pause = not self.pause
            print "pause ", self.pause
            resp1 = switch_off(self.pause)
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

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
        if key == 'update\n':
            print "uodating tar"
            self.updateTar(self.frontScan, self.rearScan);
        if key == 'save-1\n':
            print "saving scan1"
            self.saveFront = True;
            self.saveRear = True;
        if key == 'save-2\n':
            print "saving scan2"
            self.saveFrontTwo = True;
            self.saveRearTwo = True;
        if key == 'pause\n':
            print "pausing registration.."
            self.sendPause()
        if key == 'test\n':
            print "action test"
            tuck_arm_client = actionlib.SimpleActionClient('registration_fine_adjust', RegistrationFineAdjustAction)
            goal = RegistrationFineAdjustGoal()
            goal.front = self.frontScan
            goal.rear = self.rearScan
            goal.x_thresh = 0.005
            goal.y_thresh = 0.005
            goal.theta_thresh = 0.0005
            goal.duration = 30;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"
        if key == 'scan-test\n':
            print "scan test"
            tuck_arm_client = actionlib.SimpleActionClient('registration_fine_adjust', RegistrationFineAdjustAction)
            goal = RegistrationFineAdjustGoal()
            goal.front = self.frontScan
            goal.rear = self.rearScan
            goal.x_thresh = 0.005
            goal.y_thresh = 0.005
            goal.theta_thresh = 0.0005
            goal.duration = 30;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"
            print "next tar"
            goal = RegistrationFineAdjustGoal()
            goal.front = self.frontScanTwo
            goal.rear = self.rearScanTwo
            goal.x_thresh = 0.005
            goal.y_thresh = 0.005
            goal.theta_thresh = 0.0005
            goal.duration = 30;
            tuck_arm_client.wait_for_server(rospy.Duration(10.0))
            print "waiting start"
            tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(10.0))
            print "result ", tuck_arm_client.get_result()
            print "state ", tuck_arm_client.get_state()
            print "waiting done"
    
            

if __name__ == "__main__":
    fa = fakeMonitor()
    fa.loop()


