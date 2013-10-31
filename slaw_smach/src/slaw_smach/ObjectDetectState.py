#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_smach')

import rospy
import tf

import math
import actionlib

import smach
import copy
import numpy as np

from slaw_arm_navigation.msg import *

from slaw_vision.msg import *

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal

from actionlib_msgs.msg import *
from std_msgs.msg import String
#from geometry_msgs.msg import *
from slaw_manipulation.msg import PoseStampedLabeled
from slaw_navigation.msg import *

#CONFIDENCE = 93
TIME_DIFF = 0.2
base_frame = '/arm_base_link'



class ScanForObjectCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        rospy.Subscriber('/vision_detect', String, self.cb_detect)
        self.is_active = False
        self.is_done = False
        
    def cb_detect(self, msg):
        self.current_object = msg
        if not self.is_active:
            return
        
        if self.current_object.data == 'detected':
            self.is_done = True

    def execute(self, userdata):
        self.is_active = True
        self.is_done = False

        r = rospy.Rate(30)
        while not self.is_done:
            r.sleep()

        return 'success'

class ScanForObjectsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed', 'nothing_found'], input_keys=['pose_in'], output_keys=['pose_out', 'object_out', 'point_out'])
        #self.tfListen = tf.TransformListener()
        #listen for some time
        #self.classes = rospy.get_param("classes")
        #self.objectLocations = {}

        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        self.odom_fine_client.wait_for_server()
        
        self.current_object = None
        rospy.Subscriber('/vision/detected_object', PoseStampedLabeled, self.cb_detect)
        #self.copying = False
        #rospy.sleep(1)
        self.counter = 0
        
    def cb_detect(self, msg):
        self.current_object = msg


    def acceptObject(self, objects):
        now = rospy.Time.now()
        if self.current_object is None or (now-self.current_object.pose.header.stamp).to_sec() > 0.5:
            return False
        found = False

        if self.current_object.label in objects:
            found = True

        #elif self.current_object.label in ['V20', 'R20']:
        #    if 'V20' in objects or 'R20' in objects:
        #        found = True
                
        if not found:
            return False
        
        if self.current_object.pose.pose.position.y < 0.1 and self.current_object.pose.pose.position.y > -0.07:
            self.counter += 1
            if self.counter > 10:
                return True
        return False
    
    def execute(self, userdata):
        pose = userdata.pose_in
        userdata.pose_out = pose
        self.counter = 0
        param_pose = rospy.get_param(pose)
        param_objects = param_pose['objects']

        side = param_pose['side']
        
        objects = [x['name'] for x in param_objects]
        print "OBJECTS AT LOCATION :", objects
        #best = self.getBestObject(objects)

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = -0.05
        else:
            goal.target.y = -0.05
        #goal.threshold.x = 0.05
        #goal.threshold.y = 0.05
        #goal.threshold.theta = 1 # this doesn't matter
        goal.max_dist = 0.45
        goal.duration = 11

        self.odom_fine_client.send_goal(goal)

        r = rospy.Rate(20)


    
        while not self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED and not self.acceptObject(objects):
            # move
            r.sleep()

        if self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED:
            print "MOVED"
            #userdata.pose_out = "D0"
            return "nothing_found"

        # stop
        for i in xrange(5):
            self.odom_fine_client.cancel_all_goals()
        rospy.sleep(0.5)

        if self.current_object.label in ['V20', 'R20']:
            if 'V20' in objects:
                userdata.point_out = self.current_object.pose.pose
                userdata.object_out = 'V20'
                return 'success'
            if 'R20' in objects:
                userdata.point_out = self.current_object.pose.pose
                userdata.object_out = 'R20'
                return 'success'

            
        if not self.current_object.label in objects:
            return "failed"

        
        userdata.point_out = self.current_object.pose.pose
        userdata.object_out = self.current_object.label
        return "success"




class ScanForHoles(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed', 'nothing_found'], input_keys=['pose_in', 'object_in'], output_keys=['object_out', 'pose_out', 'point_out'])
        #self.tfListen = tf.TransformListener()
        #listen for some time
        #self.classes = rospy.get_param("classes")
        #self.objectLocations = {}

        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
        self.odom_fine_client.wait_for_server()
        
        self.current_object = None
        rospy.Subscriber('/vision/detected_hole', PoseStampedLabeled, self.cb_detect)
        #self.copying = False
        #rospy.sleep(1)
        
    def cb_detect(self, msg):
        self.current_hole = msg


    def acceptHole(self, hole):
        now = rospy.Time.now()
        if self.current_hole is None or (now-self.current_hole.pose.header.stamp).to_sec() > 0.5:
            return False
        if not self.current_hole.label == hole:
            return False
        if self.current_hole.pose.pose.position.y < 0.14 and self.current_hole.pose.pose.position.y > -0.06:
            return True
        return False
    
    def execute(self, userdata):
        pose = userdata.pose_in
        userdata.pose_out = pose
        obj = userdata.object_in
        userdata.object_out = obj

        param_obj = rospy.get_param(obj)
        hole = param_obj['hole']

        param_pose = rospy.get_param(pose)
        side = param_pose['side']
        
        #best = self.getBestObject(objects)

        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = -0.05
        else:
            goal.target.y = -0.05
        #goal.threshold.x = 0.05
        #goal.threshold.y = 0.05
        #goal.threshold.theta = 1 # this doesn't matter
        goal.max_dist = 0.45
        goal.duration = 11

        self.odom_fine_client.send_goal(goal)

        r = rospy.Rate(20)
        
        while not self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED and not self.acceptHole(hole):
            # move
            r.sleep()

        if self.odom_fine_client.get_state() == GoalStatus.SUCCEEDED:
            print "MOVED"
            #userdata.pose_out = "D0"
            return "nothing_found"

        # stop
        for i in xrange(5):
            self.odom_fine_client.cancel_all_goals()
        rospy.sleep(0.5)
        
        if not self.current_hole.label == hole:
            return "failed"

        userdata.point_out = self.current_hole.pose.pose

        return "success"

    
class DetectObjectsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in', 'object_in'], output_keys=['pose_out', 'object_out', 'point_out'])

        self.detect_pose = None    
        rospy.Subscriber("/vision/detected_object", PoseStampedLabeled, self.cb_detect)

    def cb_detect(self,msg):
        self.detect_pose = msg

    def execute(self, userdata):
        rate = rospy.Rate(10)
        for i in xrange(10): 
            if (rospy.Time.now() - self.detect_pose.header.stamp).to_sec() < TIME_DIFF :
                userdata.pose_out = userdata.pose_in
                userdata.object_out = userdata.object_in

                userdata.point_out = self.detect_pose.pose
                return 'success'
        return 'failed'

    
    

class KinectDetectionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out', 'object_out', 'point_out'])
        self.tfListen = tf.TransformListener()
        #listen for some time
        self.classes = rospy.get_param("classes")
        self.objectLocations = {}
                
       
        self.targetArray = rospy.Subscriber('/vision/targets', TargetArray, self.cbTargetArray)
        self.copying = False
        rospy.sleep(1)

        

    def cbTargetArray(self, msg):
        #print msg
        for t in msg.targets:
            pose_in = PoseStamped()
            pose_in.pose = t.pose
            pose_in.header.frame_id = msg.header.frame_id
            obj_name = self.classes[t.identifier]

            
            if self.tfListen.frameExists(pose_in.header.frame_id) and self.tfListen.frameExists(base_frame):
                time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, base_frame)
                pose_in.header.stamp = time
                pose = self.tfListen.transformPose(base_frame, pose_in)

                #while not self.copying:
                self.objectLocations[obj_name] = {}
                self.objectLocations[obj_name]['pose'] = pose
                self.objectLocations[obj_name]['confidence'] = t.confidence
                self.objectLocations[obj_name]['time'] = time
                    #rospy.sleep(0.0001)


    def getBestObject(self, objects):
        self.copying = True
        tmpDict = copy.deepcopy(self.objectLocations)
        print tmpDict
        
        self.copying = False
        best = None
        conf = 0
        now = rospy.Time.now()
        for obj in objects:
            if obj in tmpDict.keys():
                if tmpDict[obj['name']]['confidence']>conf and  (now-tmpDict[obj['name']]['time']).to_sec() < TIME_DIFF:
                    conf = tmpDict[obj['name']]['confidence']>conf
                    best = obj
        return best
                
    def execute(self, userdata):
        pose = userdata.pose_in
        pos = rospy.get_param(pose)
        userdata.pose_out = pose

        objects = pos['objects']

        best = self.getBestObject(objects)

        print best

        if best is None:
            return 'failed'

        object_name = best['name']
        print self.objectLocations
        userdata.point_out = self.objectLocations[object_name]['pose']
        userdata.object_out = object_name

        return 'success'

