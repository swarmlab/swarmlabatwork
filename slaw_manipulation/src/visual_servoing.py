#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_vision')
roslib.load_manifest('slaw_manipulation')
import rospy
import actionlib

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

from slaw_manipulation.msg import *

from slaw_manipulation.grip_and_place import GripAndPlace

from cv_bridge import CvBridge, CvBridgeError

import thread
import sys
import cv
import cv2
import numpy as np
import random
from math import sin, cos, pi

MAX_LIN_VEL = 0.2
LIN_VEL = 0.05
CROSS_OFF_X = 1
CROSS_OFF_Y = 18
MAX_OFF = 0

MIN_DIST_X = 0.05
MIN_DIST_Y = 0.02 # TODO

ROT_OFF_F = 0.00 # 0.04 TODO
ROT_OFF_R = 0.00

LASER_2_WHEEL_F = 0.17
LASER_2_WHEEL_R = 0.17
LASER_OFF_F = 0.03
LASER_OFF_R = 0.03

MAX_COUNT = 200 # TODO check if that makes sense

SCAN_FRONT = 0.05
SCAN_REAR = 0.40

MIN_DIFF = 0.005
LEFT = 1
RIGHT = -1

def get_dist(angle, msg, n=2):
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


class VisualServoing():

    def __init__(self):
        self.bridge = CvBridge()

        self.fleft = 0
        self.fright = 0
        self.front = 0

        self.rleft = 0
        self.rright = 0
        self.rear = 0
        
        self.reset()

        self.fix_count = 0

        self.angle = None
        self.last_detect = ""
        
        # don't allow arm movement
        self.is_done = True
        self.is_failed = True

        # init grasping
        rospy.loginfo("starting up gripping ...")
        self.gripping = GripAndPlace()
        #rospy.loginfo("moving to pre grasp position")
        rospy.loginfo("starting image subscription")
        rospy.Subscriber("/usb_cam/image_raw", Image, self.cb_image)

        # TODO maybe move this to navigation at some point
        rospy.Subscriber("/base_scan_front", LaserScan, self.cb_front)
        rospy.Subscriber("/base_scan_rear", LaserScan, self.cb_rear)
        self.pub_twist = rospy.Publisher("/cmd_vel", Twist)
        
        # start control thread
        thread.start_new_thread(self.twist_thread, ())

        self.action_server = actionlib.simple_action_server.SimpleActionServer('visual_servoing', VisualServoingAction, self.cb_execute)
        
    def reset(self):
        self.is_positioning = False # sending twist to youBot
        self.is_grasping = False
        self.is_done = False
        self.is_failed = False
        self.is_scanning = False

        self.initial_front = None
        self.scan_dir = 0
        
        self.xvel = 0
        self.yvel = 0

        self.is_only_scanning = False
        self.first_location = None
        
    def cb_execute(self, goal):
        rospy.loginfo("got an action call ...")

        self.reset()
        self.is_done = True
# start positioning

        #max_iterations = goal.iterations
        #distance = goal.distance
        self.position = goal.position
        
        result = VisualServoingResult()

        if not self.position == "left":
            rospy.logerr("can't grasp for position: %s"%self.position)
            result.result = False
            self.action_server.set_aborted(result)

        # pre grip location
        self.gripping.pre_grip() # TODO position left/right
        rospy.sleep(2)

        self.is_positioning = True


        # TODO TODO TODO TODO make goal
        self.is_only_scanning = goal.only_scanning
        self.counter = 0
        

        self.is_done = False
        r = rospy.Rate(5)
        while not self.is_done:
            r.sleep()
            if self.action_server.is_preempt_requested():
                rospy.logerr("Is preempted")
                result.result = False # failed
                self.action_server.set_preempted(result)
                self.reset()
                return

        if self.is_failed:
            rospy.logerr("failed!")
        else:
            rospy.loginfo("success!")

        result.result = not self.is_failed
        result.object = self.last_detect
        if result.result:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(result)

    def cb_front(self, msg):
        #    rospy.loginfo("got front laser msg")
        self.fleft = get_dist(pi/2 - ROT_OFF_F, msg) - LASER_2_WHEEL_F
        self.fright = get_dist(-pi/2 - ROT_OFF_F, msg) - LASER_2_WHEEL_F
        self.front = get_dist(0- ROT_OFF_F, msg) - LASER_OFF_F
        #rospy.loginfo("left: %f\t\tright: %f\t\tfront: %f"%(self.fleft, self.fright, self.front))
    
    def cb_rear(self, msg):
        #    rospy.loginfo("got rear laser msg")
        self.rleft = get_dist(-pi/2 - ROT_OFF_R, msg) - LASER_2_WHEEL_R
        self.rright = get_dist(pi/2 - ROT_OFF_R, msg) - LASER_2_WHEEL_R
        self.rear = get_dist(0 - ROT_OFF_R, msg) - LASER_OFF_R
        #rospy.loginfo("\t\t\t\t\t\t\t\t\tleft: %f\t\tright: %f\t\trear: %f"%(self.rleft, self.rright, self.rear))
        
    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv(msg, "bgr8")
            frame = np.array(frame, dtype=np.uint8)
            self.process_image(frame)
            key = cv.WaitKey(5)
            #print key
            if key == 27:
                # stop
                self.is_positioning = False
                self.is_gripping = False
                self.is_done = True
                self.is_failed = True
                
            if key == ord('g'):
                # manual grip
                self.gripping.grip()
                rospy.sleep(1)
            if key == ord('p'):
                # manual grip
                self.gripping.place(height=0.055)
                rospy.sleep(1)
            if key == ord('o'):
                # manual grip
                self.gripping.place(side='right',height=0.055)
                rospy.sleep(1)
           
            if key == ord('f'):
                # manual pre grip
                self.gripping.pre_grip()
                rospy.sleep(1)
            if key == ord('u'):
                # arm up
                self.gripping.armTuck()
                rospy.sleep(1)
            if key == ord('r'):
                # restart
                self.is_failed = True
                self.is_done = True
                rospy.sleep(3)
                                
                goal = VisualServoingGoal()
                goal.position = "left"

                c = actionlib.SimpleActionClient("visual_servoing", VisualServoingAction)
                c.wait_for_server()
                c.send_goal(goal)


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

        cv2.bitwise_not(frame_filter, frame_filter)

        kernel = np.ones((3,3),'uint8')
        frame_dilate = cv2.dilate(frame_filter, np.array((3,3)))
        #cv2.imshow("camera_dilate", frame_dilate)


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

        self.angle = None
        if largest is not None and len(largest) >= 5:
            self.is_scanning = False
            cv2.drawContours(frame, [largest], -1, (255,0,0), 1)

            ellipse = cv2.fitEllipse(largest)
            cv2.ellipse(frame,ellipse,(0,255,0),2)

            center = tuple(np.int32(ellipse[0]))
            axis = tuple(np.int32(ellipse[1]))
            
            self.angle = ellipse[-1] / 180.0 * pi
            
            cv2.circle(frame, center, 3, (0,0,255), 2)
            r = 30.0
            cv2.line(frame, 
                     tuple(np.int32(center) + np.int32([r*cos(self.angle),r*sin(self.angle)])),
                     tuple(np.int32(center) - np.int32([r*cos(self.angle),r*sin(self.angle)])),
                     (0,0,255),
                     2)

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

            xdist = center[0] - cross_w
            ydist = center[1] - cross_h

            self.set_dist(xdist, ydist)

            if self.is_only_scanning:
                if self.first_location is not None and abs(self.front - self.first_location) < 0.06:
                    # print backup
                    self.set_speed(-0.1, 0) # backup
                    self.fix_count = 0

                elif abs(xdist) <= MAX_OFF:
                    #we got a fix
                    if max(axis[0], axis[1]) > 80:
                        self.last_detect = "long"
                    else:
                        self.last_detect = "short"

                    self.fix_count += 1
                    if self.first_location is None and self.fix_count > 5:
                        self.last_detect = "long"
                        rospy.logerr("!!!!!!!!!!!!!!!!!!!!!!!  found hole: %s"%self.last_detect)
                        rospy.set_param("/positions/PPT%s/front_offset"%self.last_detect, self.front)
                        self.first_location = self.front
                        self.fix_count = 0
                    elif self.fix_count > 5:
                        self.last_detect = "short"
                        rospy.logerr("!!!!!!!!!!!!!!!!!!!!!! found second hole: %s"%self.last_detect)
                        rospy.set_param("/positions/PPT%s/front_offset"%self.last_detect, self.front)
                        self.gripping.armTuck()
                        rospy.sleep(1)
                        self.is_only_scanning = False
                        self.is_positioning = False
                        self.is_done = True
                        self.is_failed = False
                        
            elif abs(xdist) <= MAX_OFF and abs(ydist) <= MAX_OFF:
                # we got a fixs
                self.fix_count += 1
                if not self.is_done and not self.is_grasping and self.fix_count > 5:
                    self.fix_count = 0
                    rospy.loginfo("we got a fix, offset: (%f, %f)"%(xdist, ydist))
                    # long 90, 24
                    if max(axis[0], axis[1]) > 75:
                        self.last_detect = "long"
                    else:
                        self.last_detect = "short"
                    self.is_positioning = False

                    
                    self.is_grasping = True
                    self.grasp(self.angle)
                        
        else:
            self.is_scanning = True
#            rospy.loginfo("didn't find object, scanning ...")
            if self.initial_front is None:
                self.initial_front = self.front
                rospy.loginfo("inital front distance: %f"%self.initial_front)
                self.scan_dir = 1

            if self.scan_dir > 0:
                #rospy.loginfo("scan forward")
                if self.front < self.initial_front - SCAN_FRONT or self.front < MIN_DIST_X:
                    self.scan_dir = - self.scan_dir
                    rospy.loginfo("switch scanning direction")
            if self.scan_dir < 0:
                #rospy.loginfo("scan backward")
                if self.front > self.initial_front + SCAN_REAR:
                    self.scan_dir = 0
                    rospy.loginfo("completed scanning")

                    self.gripping.armTuck()
                    rospy.sleep(1)
                    
                    self.is_positioning = False
                    self.is_scanning = False
                    self.is_done = True
                    self.is_failed = True
                
            self.set_speed(0.1*self.scan_dir, 0) #forward
                    
        # show camera image with annotations
        cv2.imshow("camera contours", frame)

                    
    def grasp(self, angle):
        rospy.loginfo("performing grasp at angle: %f"%angle)
        
        pose = [x for x in self.gripping.configuration]
        gripper_pos = 2.96 - angle

        # wrap angle
        if gripper_pos < 0.115:
            gripper_pos += pi
        if gripper_pos > 5.64:
            gripper_pos -= pi

        pose[4] = gripper_pos
                    
        rospy.loginfo("rotate gripper_pos to %f"%gripper_pos)
        self.gripping.go([pose])

        #rospy.sleep(1)
        rospy.loginfo("grasping ...")
        self.gripping.grip()

        rospy.loginfo("arm up ...")        
        self.gripping.armTuck()
                
        # we are done
        self.is_grasping = False
        self.is_done = True
        
    def set_dist(self, xdist, ydist):
        x = 0
        y = 0
    
        if abs(xdist) > MAX_OFF:
            if xdist > 0:
                x = LIN_VEL # forward
            else:
                x = -LIN_VEL # backward

        if abs(ydist) > MAX_OFF:
            if ydist > 0:
                y = -LIN_VEL # right
            else:
                y = LIN_VEL # left
        self.set_speed(x,y)
                
    def set_speed(self, x, y):
        # safety
        if self.initial_front is not None and self.front < self.initial_front - SCAN_FRONT:
            x = min(x, 0)
        if self.initial_front is not None and self.front > self.initial_front + SCAN_REAR:
            x = max(x, 0)
        if self.front < MIN_DIST_X:
            rospy.loginfo("can't go foreward")
            x = min(x, 0)
        if self.rear < MIN_DIST_X:
#            rospy.loginfo("can't go backward")
            x = max(x, 0)
        if self.fleft < MIN_DIST_Y or self.rleft < MIN_DIST_Y:
#            rospy.loginfo("can't go left")
            y = min(y, 0)
        if self.fright < MIN_DIST_Y or self.rright < MIN_DIST_Y:
#            rospy.loginfo("can't go right")
            y = max(y, 0)
        self.xvel = x
        self.yvel = y

    
    def get_rot(self):
        # case for left
        dif = self.fleft - self.rleft

        #print dif
        
        max_dif = 0.055
        min_dif = 0.005

        max_speed = 0.2
        min_speed = 0.1

        speed = (dif-min_dif)/(max_dif-min_dif)*(max_speed-min_speed)+min_speed
        speed = max(speed, min_speed)        
        speed = min(speed, max_speed)

        rot = 0
        # print speed
        if abs(dif) > MIN_DIFF:
            if dif < 0:
                rot = speed * RIGHT
            else:
                rot = speed * LEFT
        # check laser bumper
        if self.fleft < MIN_DIST_Y or self.rright < MIN_DIST_Y:
            # don't rotate left
             rot = min(rot, 0)
        if self.rleft < MIN_DIST_Y or self.fright < MIN_DIST_Y:
            # don't rotate right
            rot = max(rot, 0)

        return 0 # bad!!! rot

        
    def twist_thread(self): # started as a thread
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_positioning and not self.is_done:
                # rospy.loginfo("%d: sending twist with x:%f y:%f"%(self.counter, self.xvel, self.yvel))
                self.counter+=1
                if self.counter > MAX_COUNT and not self.is_only_scanning:
                    # try grasp if something is in center
                    # if self.angle is not None:
                    #     self.grasp(self.angle)
                    #     self.is_failed = False
                    #     rospy.sleep(1)
                    self.is_positioning = False
                    self.is_grasping = False
                    self.is_failed = True
                    self.is_done = True
                    rospy.logerr("failed in twist_thread")
                    continue
                self.send_twist(self.xvel, self.yvel)
                rospy.sleep(0.05)
                self.send_twist(0, 0)
                r.sleep()
        
    def send_twist(self, x, y):
        x = min(x, MAX_LIN_VEL)
        x = max(x, -MAX_LIN_VEL)
        y = min(y, MAX_LIN_VEL)
        y = max(y, -MAX_LIN_VEL)
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y

        msg.angular.z = self.get_rot()
        
        self.pub_twist.publish(msg)
        
if __name__ == "__main__":
    rospy.init_node("visual_servoing")
    vs = VisualServoing()
    rospy.sleep(1)
    
    # goal = VisualServoingGoal()
    # goal.position = "left"

    # c = actionlib.SimpleActionClient("visual_servoing", VisualServoingAction)
    # c.wait_for_server()
    # c.send_goal(goal)

    
    try:
        rospy.spin()
    except Keyboardinterrupt:
        pass

    # to be save send 0-twist
    #vs.send_twist(0, 0)
    cv.DestroyAllWindows()
    
