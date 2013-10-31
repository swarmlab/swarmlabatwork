#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_vision')
roslib.load_manifest('slaw_manipulation')
import rospy
import actionlib

from std_msgs.msg import String
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
import tf

from math import sin, cos, pi

#from geometry_msgs.msg import PoseStamped
from slaw_manipulation.msg import PoseStampedLabeled


#CROSS_OFF_X = 9
#CROSS_OFF_Y = 47


CROSS_OFF_X = 12
CROSS_OFF_Y = 50


#PIXEL_TO_CM = 12.3016

PIXEL_TO_CM = 11.7

MIN_AREA = 1000
MAX_LEN = 300
MIN_LEN = 1

DIST_BETWEEN_CONTOURS = 0.02 #in m

def pixel_to_m(pix):
    return (pix/PIXEL_TO_CM)/100

def m_to_pixel(m):
    return int(m* 100* PIXEL_TO_CM)

OBJECTS = ["unknown",  # 0
           "F20_20_B", # 1
           "F20_20_G", # 2
           "S40_40_B", # 3
           "S40_40_G", # 4
           "M20_100",  # 5
           "M20",      # 6
           "M30",      # 7
           "R20",      # 8
           "V20",      # 9
           ]

def j48(features):
    (area, axis1, axis2, intensity) = features
    if axis2 <= 73.239334:
        if intensity <= 70.743005:
            if intensity <= 39.172991:
                return "R20"
            if intensity > 39.172991:
                if intensity <= 40.306196:
                    if axis2 <= 66.370174:
                        return "V20"
                    if axis2 > 66.370174:
                        if axis2 <= 71:
                            if axis1 <= 46.872167:
                                return "R20"
                            if axis1 > 46.872167:
                                if axis1 <= 47.381431:
                                    return "V20"
                                if axis1 > 47.381431:
                                    return "R20"
                        if axis2 > 71:
                            return "V20"
                if intensity > 40.306196:
                    if axis2 <= 63.071388:
                        if area <= 2746.5:
                            return "V20"
                        if area > 2746.5:
                            return "R20"
                    if axis2 > 63.071388:
                        return "V20"
        if intensity > 70.743005:
            if axis1 <= 54.037024:
                return "M20"
            if axis1 > 54.037024:
                return "M30"
    if axis2 > 73.239334:
        if intensity <= 78.693857:
            if axis1 <= 54.037024:
                if axis1 <= 44.28318:
                    return "F20_20_B"
                if axis1 > 44.28318:
                    return "M20_100"
            if axis1 > 54.037024:
                return "S40_40_B"
        if intensity > 78.693857:
            if axis1 <= 41.231056:
                if area <= 5247:
                    if intensity <= 167.580994:
                        return "S40_40_G"
                    if intensity > 167.580994:
                        return "F20_20_G"
                if area > 5247:
                    return "S40_40_G"
    return "unknown"

class DetectObjects():
    def __init__(self):
        self.bridge = CvBridge()
        self.fix_count = 0
        self.angle = None
        self.object_type = None

#        self.detector  = cv2.FastFeatureDetector(16, True) #cv2.SURF()
        
        # init camera
        rospy.Subscriber("/usb_cam/image_raw", Image, self.cb_image)
        self.pose_pub = rospy.Publisher("/vision/detected_object", PoseStampedLabeled)

        self.detect_pub = rospy.Publisher("/vision_detect", String)
        
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

            if key >= 48 and key <= 57:
                self.object_type = OBJECTS[key - 48]
            else:
                self.object_type = None

        except CvBridgeError, e:
            print e



    # def j48(self, features):
    #     (area, axis1, axis2, intensity) = features
    #     if axis2 <= 73.171033:
    #         if intensity <= 103.583049:
    #             return "plastic_riffle"
    #         else:
    #             if axis2 <= 49:
    #                 return "nut_small"
    #             else:
    #                 return "nut_big"
    #     else:
    #         if axis1 <= 44.181444:
    #             if intensity <= 140.260331:
    #                 return "alu_black"
    #             else:
    #                 return "alu_silver"
    #         else:
    #             return "screw"
    #     return "unknown"

    def filter_contours(self, contours):

        conts = []
        for c in contours:
            if len(c) <= 4:
                continue
            area = cv2.contourArea(c)
            #print  area
            if area > MIN_AREA:

                rect = cv2.minAreaRect(c)
                box = cv2.cv.BoxPoints(rect)
                box = np.int32(box)
                vec1 = box[1] - box[2]
                vec2 = box[2] - box[3]


                axis = (np.linalg.norm(vec1), np.linalg.norm(vec2))

                #ellipse = cv2.fitEllipse(c)
                #axis = tuple(np.int32(ellipse[1]))
                #print axis
                if (max(axis) < MAX_LEN) and (min(axis) > MIN_LEN):
                    #print axis, area
                    conts.append(cv2.convexHull(c))

        done = False#False #HENNES: True = no merge
        #print conts
        #print len(contours)
        #print len(conts)

        while not done:
            done = True
            for idx,cnt in enumerate(conts):
                M = cv2.moments(cnt)
                #ellipse = cv2.fitEllipse(c)
                #center = tuple(np.int32(ellipse[0]))


                centroid_x = int(M['m10']/M['m00'])
                centroid_y = int(M['m01']/M['m00'])

                center = (centroid_x, centroid_y)
                #print len(conts), idx
                
                for idy in range(0, len(conts)):
                    #print idy, "test"
                    if idy == idx:
                        continue
                    
                    dist = cv2.pointPolygonTest(conts[idy],center,True)
                    #print dist
                    if abs(pixel_to_m(dist)) < DIST_BETWEEN_CONTOURS:
                        done = False

                        #print "Found one", idx, idy, len(conts)
                        test = np.append(conts[idy], cnt, axis=0)
                        #print test

                        if idy>idx:
                            conts.pop(idy)
                            conts.pop(idx)
                        else:
                            conts.pop(idx)
                            conts.pop(idy)       

                        conts.append(cv2.convexHull(test))


                     
                        break
                if not done:
                    break
        #print "done"
        # conts2 = []
        # for c in conts:
        #     if len(c) <= 4:
        #         continue
        #     area = cv2.contourArea(c)
        #     if area > MIN_AREA:
        #         ellipse = cv2.fitEllipse(c)
        #         axis = tuple(np.int32(ellipse[1]))
        #         if (max(axis) < MAX_LEN):
        #             conts2.append(cv2.convexHull(c))

                    
        return conts

            
    def process_image(self, frame):
        #rospy.loginfo("%s %s %s %s"%(str(self.is_positioning), str(self.is_grasping), str(self.is_done), str(self.is_failed)))

#rospy.loginfo("processing image ...")
        #cv2.imshow("camera", frame)

        # gray
        frame_gray = cv2.cvtColor(frame, cv.CV_RGB2GRAY)
        frame_gray_copy = cv2.cvtColor(frame, cv.CV_RGB2GRAY)

        #frame_gray = cv2.blur(frame_gray, (3,3))
        #frame_gray = cv2.GaussianBlur(frame_gray, (9, 9), 0)
        #cv2.imshow("camera_gray", frame_gray)


        size = frame.shape
        #size = (size[1]-1, size[0]-1)
        

        # cv2.rectangle(frame_filter, (0,0), (640,180),
        #                255, # color
        #                cv2.cv.CV_FILLED, # thickness
        #                8, # line-type ???
        #                0) # random shit
        


        cv2.imshow("gray", frame_gray)
        (_, frame_bw) = cv2.threshold(frame_gray, 100, 255, cv.CV_THRESH_BINARY | cv.CV_THRESH_OTSU)
        cv2.imshow("bw", frame_bw)
        cv2.bitwise_not(frame_bw, frame_bw)

        #         # contours
        
        contours, hierarchy = cv2.findContours(frame_bw, 
                                               cv2.RETR_EXTERNAL, 
                                               cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(frame, contours, -1, (255,0,0), 1)

        cv2.imshow("frame", frame)

        detected = False
        
        for c in contours:
            if len(c) <= 4:
                continue
            ellipse = cv2.fitEllipse(c)
            
            #cv2.ellipse(frame,ellipse,(0,255,0),2)
            
            #rect = cv2.minAreaRect(c)
            center = np.int32(ellipse[0])
            area = cv2.contourArea(c)
            
            if center[0] > 60 and center[0] < 100 and center[1] < 40 and area > 20:
                
                detected = True
                
        if detected:
            print "detected"
            self.detect_pub.publish("detected")
        else:
            print "not"
            self.detect_pub.publish("not detected")




#         size = frame.shape
#         size = (size[1]-1, size[0]-1)
        
#         # rectangle
#         cv2.rectangle(frame_filter, (0,0), (640,180),
#                        255, # color
#                        cv2.cv.CV_FILLED, # thickness
#                        8, # line-type ???
#                        0) # random shit



#         kernel = np.ones((9,9),'uint8')
#         frame_dilate = cv2.dilate(frame_filter, kernel)


#         # # rectangle
#         # cv2.rectangle(frame_dilate, (0,0), size,
#         #               0, # color
#         #               20, # thickness
#         #               8, # line-type ???
#         #               0) # random shit





#         # contours
#         contours, hierarchy = cv2.findContours(frame_filter, 
#                                                cv2.RETR_EXTERNAL, 
#                                                cv2.CHAIN_APPROX_SIMPLE)


#         cross_w = size[0]/2 + CROSS_OFF_X
#         cross_h = size[1]/2 + CROSS_OFF_Y

#         cross = np.int32([cross_w, cross_h])
#         # crosshair
#         cv2.line(frame,
#                  (cross_w, 0),
#                  (cross_w, size[1]),
#                  (255,255,0))

#         cv2.line(frame,
#                  (0, cross_h),
#                  (size[0], cross_h),
#                  (255,255,0))


#         largest = None
#         #max_area = 0
#         smallest_dist = max(size[0], size[1])

#         #print contours

#         contours = self.filter_contours(contours)

#         cv2.drawContours(frame, contours, -1, (255,0,0), 1)
        
#         cv2.circle(frame, tuple(cross), m_to_pixel(0.08), (0,0,255), 2)

#         for c in contours:
#             ellipse = cv2.fitEllipse(c)
            
#             #cv2.ellipse(frame,ellipse,(0,255,0),2)
            
#             #rect = cv2.minAreaRect(c)
#             center = np.int32(ellipse[0])


            
#             #xdist = abs(center[0] - cross_w)
#             dist = np.linalg.norm(center - cross)

#             if dist<smallest_dist:
#                 smallest_dist = dist
#                 largest = c # HENNES: this is now closest?
                    
#                     #if max_area < area:
#                     #    max_area = area
#                     #    largest = c

#         self.angle = None
        
#         if largest is not None: #HENNES: not needed? # and len(largest) >= 5:

#             rect = cv2.minAreaRect(largest)
#             #print rect
#             box = cv2.cv.BoxPoints(rect)
#             #print box
#             box = np.int32(box)
#             cv2.drawContours(frame,[box],0,(0,0,255),2)
            
#             #cv2.ellipse(rect,ellipse,(0,255,0),2)
                        
#             ellipse = cv2.fitEllipse(largest)
            
#             center = tuple(np.int32(ellipse[0]))
#             axis = tuple(np.int32(ellipse[1]))
            
#             cv2.circle(frame, tuple(center), 3, (0,0,255), 2)
#             vec1 = box[1] - box[2]
#             vec2 = box[2] - box[3]

#             # FEATURES
#             # TODO: all features should be calculated on eroded image            
#             mask = np.zeros(frame_gray.shape, dtype = np.uint8)
#             cv2.drawContours(mask, [largest], 0 ,255, -1)

#             kernel = np.ones((11,11),'uint8')
#             mask_erode = cv2.erode(mask, kernel)

# #            # get object from gray image with smaller mask
# #            object_image = frame_gray_copy.copy()
# #            object_image[mask_erode == 0] = 0
# #            cv2.imshow("object", object_image)            

#             mean_val = cv2.mean(frame_gray_copy, mask = mask_erode)
#             intensity = mean_val[0]

#             axis2 = (np.linalg.norm(vec1), np.linalg.norm(vec2))
#             feature_vector = [cv2.contourArea(largest), min(axis2), max(axis2), intensity]

#             object_label = "unknown"
#             if self.object_type:
#                 print ",".join(str(x) for x in feature_vector) + "," + self.object_type
#             else:
#                 object_label = j48(feature_vector)
#                 print object_label
#             # if object_label == "R20V20":
#             #     if intensity > 38:
#             #         print "V20"
#             #     else:
#             #         print "R20"
            
#             if np.linalg.norm(vec1) < np.linalg.norm(vec2):
#                 self.angle = np.arctan2(vec1[1], vec1[0])
#             else:
#                 self.angle = np.arctan2(vec2[1], vec2[0])
                
#             r = 30.0
#             cv2.line(frame, 
#                      tuple(np.int32(center) + np.int32([r*cos(self.angle),r*sin(self.angle)])),
#                      tuple(np.int32(center) - np.int32([r*cos(self.angle),r*sin(self.angle)])),
#                      (0,0,255),
#                      2)

#             xdist = center[0] - cross_w
#             ydist = center[1] - cross_h

            
#             x_m = pixel_to_m(xdist)
#             y_m = pixel_to_m(ydist)


#             msg = PoseStampedLabeled()
#             msg.pose.header.frame_id = '/arm_base_link'
#             msg.pose.header.stamp = rospy.Time.now()

#             #double check
#             msg.pose.pose.position.x = y_m
#             msg.pose.pose.position.y = -x_m

#             quat = tf.transformations.quaternion_from_euler(0,0,self.angle)
            
#             msg.pose.pose.orientation.x = quat[0]
#             msg.pose.pose.orientation.y = quat[1]
#             msg.pose.pose.orientation.z = quat[2]
#             msg.pose.pose.orientation.w = quat[3]
#                 #print xdist, ydist
#             #print self.angle

#             msg.label = object_label
            
#             self.pose_pub.publish(msg)
                    
#         # show camera image with annotations
#         cv2.imshow("camera contours", frame)

                    
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
    
