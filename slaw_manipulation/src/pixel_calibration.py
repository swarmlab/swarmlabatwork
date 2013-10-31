#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_vision')
roslib.load_manifest('slaw_manipulation')
import rospy
import actionlib

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import numpy as np

import cv
import cv2


class PixelCalibration():

    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cb_image)
        cv.NamedWindow('a_window', 1)
        self.x1 = 640/4*2
        self.y1 = 480/2
        self.x2 = 640/4*3
        self.y2 = 480/2
        self.offset = 5

    def cb_image(self, msg):
        try:
#            print "x1 ", self.x1, ", y1 ", self.y1, ", x2 ", self.x2, ", y2 ", self.y2 
            print "x dist: ", self.x2-self.x1

            cv_image = self.bridge.imgmsg_to_cv(msg, "bgr8")

            (cols,rows) = cv.GetSize(cv_image)
            
            cv.Circle(cv_image, (self.x1,self.y1), 5, 255)
            cv.Circle(cv_image, (self.x2,self.y2), 5, 0)
                
            cv.ShowImage('a_window', cv_image) 

            key = cv.WaitKey(1)
                
            if key == ord('f'):
                self.x1+=self.offset;

            if key == ord('e'):
                self.y1+=self.offset

            if key == ord('s'):
                self.x1-=self.offset

            if key == ord('d'):
                self.y1-=self.offset


            if key == ord('l'):
                self.x2+=self.offset

            if key == ord('i'):
                self.y2+=self.offset

            if key == ord('j'):
                self.x2-=self.offset

            if key == ord('k'):
                self.y2-=self.offset
    
            if key == ord('+'):
                offset += 1
                print "offset ", self.offset

            if key == ord('-'):
                offset -= 1
                print "offset ", self.offset

        except CvBridgeError, e:
            print e


if __name__ == "__main__":
    rospy.init_node("pixel_calibration")
    cal = PixelCalibration()
    rospy.sleep(1)
    
    try:
        rospy.spin()
    except Keyboardinterrupt:
        pass

    cv.DestroyAllWindows()
    
