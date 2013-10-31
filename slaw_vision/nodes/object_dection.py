#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_vision')

import rospy
import tf

import math
import actionlib

import numpy as np

from geometry_msgs.msg import *
from slaw_vision.msg import *

map_frame = '/map'
#base_frame = '/arm_link_0'
end_effector_frame = '/arm_link_5'

CONFIDENCE = 93

class ObjectDetection:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        #listen for some time
        rospy.sleep(2)

        self.targetArray = rospy.Subscriber('/vision/targets', TargetArray, self.cbTargetArray)
        self.poses_pub = rospy.Publisher('/vision/found', PoseArray)
        self.polygon_pub = rospy.Publisher('/vision/rect', PolygonStamped)

        self.gotPose = False
        self.stopRecPose = False
        self.poses = {}
        self.action_server = actionlib.simple_action_server.SimpleActionServer('get_objects', GetObjectsAction, self.executeCB)



    def executeCB(self, goal):
        class = goal.class
        position = goal.position

        result = GetObjectsResults()
        
        
        if class == 'any':
            result.pose

    def getBestPose(self):
        for key in self.poses:
            self.poses[key]
            
    
    def cbTargetArray(self, msg):
        for idx,t in enumerate(msg.targets):
            if t.confidence>CONFIDENCE:
                pose_in = PoseStamped()
                pose_in.pose = t.pose
                pose_in.header.frame_id = msg.header.frame_id
                identifier = t.identifier
                
                if self.tfListen.frameExists(pose_in.header.frame_id) and self.tfListen.frameExists(map_frame):
                    time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, map_frame)
                    pose_in.header.stamp = time

                    polygon = PolygonStamped()

                    polygon.header = msg.header
                    #polygon.polygon = t.hull
                    polygon.polygon = t.rectangle
                    self.polygon_pub.publish(polygon)

                    vects = []
                    for p,idx in zip(polygon.polygon.points, range(len(polygon.polygon.points))):
                        idx_two = idx+1
                        if idx_two == len(polygon.polygon.points):
                            idx_two = 0
                        p2 = polygon.polygon.points[idx_two]
                        vects.append(np.array([p2.x - p.x, p2.y - p.y]))

                    max_len = 0
                    max_vect = None
                    for vect in vects:
                        if np.linalg.norm(vect)>max_len:
                            max_vect = vect
                            max_len = np.linalg.norm(vect)
                    if max_vect is not None:
                        ang = math.atan2(max_vect[1], max_vect[0])
                        #r, p, y = tf.transformations.euler_from_quaternion(pre_left_down[1])
                        quat = tf.transformations.quaternion_from_euler(0,0,ang)
                    
                        if not self.stopRecPose:
                            newPose = {}
                            pose = self.tfListen.transformPose(map_frame, pose_in)
                            pose.pose.orientation.x = quat[0]
                            pose.pose.orientation.y = quat[1]
                            pose.pose.orientation.z = quat[2]
                            pose.pose.orientation.w = quat[3]
                            newPose['pose'] = pose
                            newPose['confidence'] = t.confidence
                            newPose['lastTime'] = msg.header.stamp
                            self.poses[identifier] = newPose
                            self.gotPose = True
                            
                    self.publishFoundPoses()
                    
                else:
                    print "tf_lookup failed"

    def publishFoundPoses(self):
        print self.poses
        if self.gotPose:
            msg = PoseArray()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = map_frame
            for key in self.poses:
                msg.poses.append(self.poses[key]['pose'].pose)
            self.poses_pub.publish(msg)
            #print msg

def main():
    rospy.init_node("ObjectDetection")
    rospy.sleep(0.001)  # wait for time
    obj_detec = ObjectDetection()
    rospy.spin()

if __name__ == '__main__':
    main()

