#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_vision')
roslib.load_manifest('slaw_manipulation')
import rospy
import actionlib

from sensor_msgs.msg import Image

from slaw_manipulation.msg import *

#from slaw_manipulation.grip_and_place import GripAndPlace

from cv_bridge import CvBridge, CvBridgeError

import thread
import sys
import cv
import cv2
import numpy as np
import random
from math import sin, cos, pi
import math

CROSS_OFF_X = 0
CROSS_OFF_Y = 0



class DetectObjects():
    def __init__(self):
        self.bridge = CvBridge()

        self.fix_count = 0

        self.angle = None

        # init camera
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_image)

        
    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            self.process_image(frame)
            key = cv.WaitKey(5)
            #print key
            if key == ord('r'):
                # restart
                rospy.sleep(3)

        except CvBridgeError, e:
            print e


    def process_image(self, frame):
        #rospy.loginfo("%s %s %s %s"%(str(self.is_positioning), str(self.is_grasping), str(self.is_done), str(self.is_failed)))

#rospy.loginfo("processing image ...")
        #cv2.imshow("camera", frame)

        # gray
        frame_gray = cv2.cvtColor(frame, cv.CV_RGB2GRAY)
        #frame_gray = cv2.blur(frame_gray, (3,3))
        frame_gray = cv2.GaussianBlur(frame_gray, (9, 9), 0)
        #cv2.imshow("camera_gray", frame_gray)

        # adaptive filter
        frame_filter = cv2.adaptiveThreshold(frame_gray, 
                                             255.0, 
                                             cv.CV_THRESH_BINARY, 
                                             #cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv.CV_ADAPTIVE_THRESH_MEAN_C,
                                             9,  # neighbourhood
                                             9)

        #cv2.imshow("camera_filter", frame_filter)
        size = frame.shape
        size = (size[1]-1, size[0]-1)

        # rectangle
        cv2.rectangle(frame_filter, (0,0), size,
                      255, # color
                      20, # thickness
                      8, # line-type ???
                      0) # random shit

        
        # rectangle
        cv2.rectangle(frame_filter, (0,0), (640,180),
                      255, # color
                      cv2.cv.CV_FILLED, # thickness
                      8, # line-type ???
                      0) # random shit


        cv2.bitwise_not(frame_filter, frame_filter)

        kernel = np.ones((3,3),'uint8')
        frame_dilate = cv2.dilate(frame_filter, np.array((3,3)))
        cv2.imshow("camera_dilate", frame_dilate)


        # contours
        contours, hierarchy = cv2.findContours(frame_dilate, 
                                               cv2.RETR_EXTERNAL, 
                                               cv2.CHAIN_APPROX_SIMPLE)


        cross_w = size[0]/2 + CROSS_OFF_X
        cross_h = size[1]/2 + CROSS_OFF_Y
        
            #crosshair
        cv2.line(frame,
                 (cross_w, 0),
                 (cross_w, size[1]),
                 (255,255,0))

        cv2.line(frame,
                 (0, cross_h),
                 (size[0], cross_h),
                 (255,255,0))


        largest = None
        max_area = 0
        smallest_x_dist = size[0]
        snd_smallest_x_dist = size[0]

        cen1 = None
        cen2 = None
        for c in contours:
            if len(c) <= 4:
                continue
            area = cv2.contourArea(c)


            if area > 100:

                ellipse = cv2.fitEllipse(c)
                axis = tuple(np.int32(ellipse[1]))
                if (max(axis) < 300):
                    cv2.drawContours(frame, [c], -1, (255,0,0), 1)
                    cv2.ellipse(frame,ellipse,(0,255,0),2)

                    center = tuple(np.int32(ellipse[0]))

                    cv2.circle(frame, center, 3, (0,0,255), 2)


                    xdist = abs(center[0] - cross_w)

                    #print ydist
                #center = tuple(np.int32(ellipse[0]))
                #axis = tuple(np.int32(ellipse[1]))
                    if xdist<smallest_x_dist:
                        snd_smallest_x_dist = smallest_x_dist
                        smallest_x_dist = xdist
                        cen2 = cen1
                        cen1 = center
                        
                    elif xdist<snd_smallest_x_dist:
                        snd_smallest_x_dist = xdist
                        cen2 = center
                    #if max_area < area:
                    #    max_area = area
                    #    largest = c

                        
        print  np.linalg.norm((np.array(cen2) - np.array(cen1)))
        print (np.array(cen2) + np.array(cen1)) /2.
        self.angle = None
        
        if largest is not None and len(largest) >= 5:
            
            ellipse = cv2.fitEllipse(largest)
            
            center = tuple(np.int32(ellipse[0]))
            axis = tuple(np.int32(ellipse[1]))
            
            self.angle = ellipse[-1] / 180.0 * pi
            

            r = 30.0
            cv2.line(frame, 
                     tuple(np.int32(center) + np.int32([r*cos(self.angle),r*sin(self.angle)])),
                     tuple(np.int32(center) - np.int32([r*cos(self.angle),r*sin(self.angle)])),
                     (0,0,255),
                     2)

            #xdist = center[0] - cross_w
            #ydist = center[1] - cross_h

            #print xdist, ydist
            
                    
        # show camera image with annotations
        cv2.imshow("camera contours", frame)

                    
if __name__ == "__main__":
    rospy.init_node("visual_servoing")
    vs = DetectObjects()
    rospy.sleep(1)
    

    
    try:
        rospy.spin()
    except Keyboardinterrupt:
        pass

    # to be save send 0-twist
    #vs.send_twist(0, 0)
    cv.DestroyAllWindows()
    
