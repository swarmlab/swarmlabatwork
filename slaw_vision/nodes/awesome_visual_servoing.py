#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_vision')
import rospy

import sys

import cv
import cv2
import numpy as np
import random
from math import sin, cos, pi

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist

from slaw_manipulation.gripping import Gripping

import thread

bridge = CvBridge()
frame = None
gripping = None

send = False

def image_callback(msg):
    global send 
    try:
        frame = bridge.imgmsg_to_cv(msg, "bgr8")
        frame = np.array(frame, dtype=np.uint8)
        process_image(frame)
#        cv2.imshow("camera", frame)
        key = cv.WaitKey(5)
        if key == 27:
            send = not send
    except CvBridgeError, e:
        print e

def blur_change(value):
    global blur
    blur = value*2 + 1

#camera_index = 0
#capture = cv2.VideoCapture(camera_index)

cv2.namedWindow("camera_gray");
cv2.createTrackbar("blur", "camera_gray", 0, 4, blur_change);
blur = 1

filter1 = 9
def filter1_change(value):
    global filter1
    filter1 = (value+1)*2 + 1

filter2 = 9
def filter2_change(value):
    global filter2
    filter2 = value


cv2.namedWindow("camera");
cv2.namedWindow("camera_filter");
cv2.createTrackbar("neighbs", "camera_filter", 3, 10, filter1_change);
cv2.createTrackbar("offset", "camera_filter", 9, 20, filter2_change);

fix = False

def process_image(frame): #(capture):
#    retval, frame = capture.read()
    cv2.imshow("camera", frame)

    
    # gray
    frame_gray = cv2.cvtColor(frame, cv.CV_RGB2GRAY)
    #frame_gray = cv2.blur(frame_gray, (3,3))
    frame_gray = cv2.GaussianBlur(frame_gray, (blur,blur), 0)

    cv2.imshow("camera_gray", frame_gray)



    # adaptive filter
    frame_filter = cv2.adaptiveThreshold(frame_gray, 
                                         255.0, 
                                         cv.CV_THRESH_BINARY, 
                                         #cv.CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv.CV_ADAPTIVE_THRESH_MEAN_C,
                                         filter1,  # neighbourhood
                                         filter2)

    cv2.imshow("camera_filter", frame_filter)


    size = frame.shape
    size = (size[1]-1, size[0]-1)

    # rectangle
    cv2.rectangle(frame_filter, (0,0), size,
                 255, # color
                 20, # thickness
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

    largest = None
    max_area = 0
    for c in contours:
        if len(c) <= 4:
            continue
        area = cv2.contourArea(c)
        if max_area < area:
            max_area = area
            largest = c

    if largest is not None and len(largest) >= 5:
        cv2.drawContours(frame, [largest], -1, (255,0,0), 1)

        ellipse = cv2.fitEllipse(largest)
        cv2.ellipse(frame,ellipse,(0,255,0),2)

        center = tuple(np.int32(ellipse[0]))
        angle = ellipse[-1] / 180.0 * pi

        # print angle
        
        cv2.circle(frame, center, 3, (0,0,255), 2)
        r = 30.0
        cv2.line(frame, 
                 #center, 
             tuple(np.int32(center) + np.int32([r*cos(angle),r*sin(angle)])),
             tuple(np.int32(center) - np.int32([r*cos(angle),r*sin(angle)])),
#                 1, 
             (0,0,255),
             2)

        cross_w = size[0]/2 + 4
        cross_h = size[1]/2 + 18
        
    #crosshair
        cv2.line(frame,
                 (cross_w, 0),
                 (cross_w, size[1]),
                 (255,255,0))

        cv2.line(frame,
                 (0, cross_h),
                 (size[0], cross_h),
                 (255,255,0))

        xdist = center[0] - cross_w
        ydist = center[1] - cross_h

    # print xdist, ydist

        global xvel
        global yvel
        global fix
        global send
        xvel = 0
        yvel = 0

        vel = 0.1
    
        if abs(xdist) > 5:
#            fix = False
            if xdist > 0:
                xvel = vel # forward
            else:
                xvel = -vel # backward

        if abs(ydist) > 5:
#            fix = False
            if ydist > 0:
                yvel = -vel # right
            else:
                yvel = vel # left

        if abs(xdist) <= 5 and abs(ydist) <= 5:
            # we got a fix
            print angle
            if not fix:
                fix = True
                send = False
                global gripping
                pose = [x for x in gripping.configuration]
                gripper = 2.96 - angle
                if gripper < 0.115:
                    gripper += pi

                if gripper > 5.64:
                    gripper -= pi

                pose[4] = gripper

                rospy.loginfo("rotate gripper to %f"%gripper)
                gripping.go([pose])

                rospy.sleep(1)
                rospy.loginfo("grip")
                gripping.grip()
#                sys.exit("error")
                
            #send once

        
    cv2.imshow("camera contours", frame)
    


MAX_VEL = 0.2
xvel = 0
yvel = 0

def send_twist():
    global xvel
    global yvel
    global send

    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        x = xvel
        y = yvel
        
        x = min(x, MAX_VEL)
        x = max(x, -MAX_VEL)
        y = min(y, MAX_VEL)
        y = max(y, -MAX_VEL)
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        #print msg
        if send:
            print xvel, yvel
            pub_twist.publish(msg)
        rospy.sleep(0.05)
        pub_twist.publish(Twist())
        r.sleep()

pub_twist = None


def main(args):
    global pub_twist
    global gripping
    rospy.init_node("visual_servoing")

    gripping = Gripping()
    rospy.sleep(1)
    gripping.pre_grip()
    rospy.sleep(1)
    
    rospy.Subscriber("usb_cam/image_raw", Image, image_callback)
    pub_twist = rospy.Publisher("/cmd_vel", Twist)

    thread.start_new_thread(send_twist, ())

    try:
        rospy.spin()
    except Keyboardinterrupt:
        pass

    global xvel
    global yvel
    xvel = 0
    yvel = 0
    send_twist()
    cv.DestroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
    
