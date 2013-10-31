#!/usr/bin/env python       

PACKAGE = 'slaw_navigation'

import roslib;roslib.load_manifest(PACKAGE)
import rospy

from dynamic_reconfigure.server import Server
from slaw_navigation.cfg import pidConfig

from slaw_navigation.srv import *

from slaw_pid.pid import PidController

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist


class positionController(object):
 

    def __init__(self):
        rospy.init_node('pose_controller', anonymous = False)
        self.pub_twist = rospy.Publisher('/cmd_vel', Twist)
        s = rospy.Service('/scan_registration/switchOffPID', switchOff, self.switchOff)
        self.pause = False;
        self.init()

        rospy.loginfo("Controller initialized")
        self.listener()


    def switchOff(self, req):
        self.pause = req.pause
        twist = Twist()
        
        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.z = 0

        self.pub_twist.publish(twist)

        return switchOffResponse(True)

    def init(self, cutoff_angular = 0, KP_angular = 0, KI_angular = 0, KD_angular = 0, Ilimit_angular = 0, cutoff_linear_x = 0, KP_linear_x = 0, KI_linear_x = 0, KD_linear_x = 0, Ilimit_linear_x = 0, cutoff_linear_y = 0, KP_linear_y = 0, KI_linear_y = 0, KD_linear_y = 0, Ilimit_linear_y = 0, groups= None):

        self.cutoff_angular_ = cutoff_angular;
        self.cutoff_linear_x_ = cutoff_linear_x
        self.cutoff_linear_y_ = cutoff_linear_y
        
        self.lastCall_linear_ = rospy.get_time()
#        self.lastCall_angular_ = rospy.get_time()

        self.pid_linear_y_ = PidController(KP_linear_y, KI_linear_y, KD_linear_y, Ilimit_linear_y)
        self.pid_linear_x_ = PidController(KP_linear_x, KI_linear_x, KD_linear_x, Ilimit_linear_x)
        self.pid_angular_ = PidController(KP_angular, KI_angular, KD_angular, Ilimit_angular)


    def callback_pose(self, data):
        if self.pause == False or True:
            dt = float(rospy.get_time() - self.lastCall_linear_)
            self.lastcall_linear_ = rospy.get_time()
        
            error_x = data.x
            error_y = data.y
            error_theta = data.theta

            print error_theta
        
            x_cmd = 0
            y_cmd = 0
            if error_theta < self.cutoff_angular_:
                x_cmd = self.pid_linear_x_.update(error_x, dt)
                y_cmd = self.pid_linear_y_.update(error_y, dt)
            theta_cmd = self.pid_angular_.update(error_theta, dt)
        
            twist = Twist()
                
            twist.linear.x = -x_cmd
            twist.linear.y = -y_cmd
            twist.angular.z = -theta_cmd

            self.pub_twist.publish(twist)


    def callback_reconf(self,config, level):
        rospy.loginfo("""Reconfiugre Request: 
        cutoff, kp, ki, kd, ilimit
 angular: {cutoff_angular}, {KP_angular}, {KI_angular}, {KD_angular}, {Ilimit_angular}, 
linear_x: {cutoff_linear_x}, {KP_linear_x}, {KI_linear_x}, {KD_linear_x}, {Ilimit_linear_x},
linear_y: {cutoff_linear_y}, {KP_linear_y}, {KI_linear_y}, {KD_linear_y}, {Ilimit_linear_y}""".format(**config))
        self.init(**config)
        return config
    
    
    def listener(self):
        dyn_srv = Server(pidConfig, self.callback_reconf)
        rospy.Subscriber('/scan_registration/tarDir', Pose2D , self.callback_pose)
        rospy.loginfo("Controller spinning..")
        rospy.spin()    
        

if __name__ == '__main__':
    try:
        conti = positionController()
    except rospy.ROSInterruptException: pass
