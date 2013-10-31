#!/usr/bin/env python
import roslib; roslib.load_manifest('slaw_cameramount')
import rospy
import tf
from std_msgs.msg import Int32, String
from slaw_cameramount.srv import *

class CameraMount():
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.loadPositions()
        self.servo_control_pub = rospy.Publisher('/cameramount/servo/setpoint', Int32)
        self.sub = rospy.Subscriber('/cameramount/position', String, self.servoPositionCB)
        rate = rospy.Rate(10)

        self.pose_service = rospy.Service('/cameramount/getPose', CamPose, self.getCurrentPose)
        while not rospy.is_shutdown():
            self.publishTransform()
            rate.sleep()


    def getCurrentPose(self, req):
        return CamPoseResponse(self.pose)

    def servoPositionCB(self, msg):
        servo_control_msg = Int32()

        if (msg.data == 'right'):
            servo_control_msg.data = 20
        elif (msg.data == 'left'):
            servo_control_msg.data = 160
        elif (msg.data == 'center'):
            servo_control_msg.data = 90
        else:
            return
            
        #print servo_control_msg.data
        self.servo_control_pub.publish(servo_control_msg)
        self.setCurTF(msg.data)
        self.publishTransform()


    def setCurTF(self, pos):
        self.key_found = False
        if self.positions:
            if self.positions.has_key(pos):
                trans = self.positions[pos]['translation']
                rot = self.positions[pos]['rotation']
                self.pose = pos
                self.cur_pos= (trans['x'], trans['y'], trans['z'])
                self.cur_quat = (rot['x'], rot['y'], rot['z'], rot['w'])
                self.key_found = True
                return
        rospy.logwarn("Key not found: " + pos + ". Not publishing transfrom")

            
    def loadPositions(self):
        self.base_link = rospy.get_param('slaw_cameramount/base_link', '/base_link')
        self.camera_link = rospy.get_param('slaw_cameramount/camera_link', '/camera_link')
        self.positions = rospy.get_param('slaw_cameramount/positions', None)
        self.publishTF = rospy.get_param('slaw_cameramount/publish_tf', True)

        self.startPos = rospy.get_param('slaw_cameramount/start_pos', 'center')
        
        self.setCurTF(self.startPos)
      

    def publishTransform(self):
        if (self.publishTF) and (self.key_found):
#            self.tf_broadcaster.sendTransform(self.cur_pos, self.cur_quat, rospy.Time.now()+rospy.Duration(0.1), self.base_link, self.camera_link)
            self.tf_broadcaster.sendTransform(self.cur_pos, self.cur_quat, rospy.Time.now()+rospy.Duration(0.1), self.camera_link, self.base_link)
            
if __name__=='__main__':
    rospy.init_node('cameramount')
    try:
        cameraHandler = CameraMount()
    except rospy.ROSInterruptException: 
        pass
