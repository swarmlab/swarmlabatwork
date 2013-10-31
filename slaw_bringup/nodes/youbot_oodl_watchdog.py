#!/usr/bin/env python
import roslib; roslib.load_manifest('slaw_bringup')
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Empty
#import subprocess

motor_on = True

def cb_diagnostic(msg):
    global motor_on;
    if msg.status[0].name == 'platform_Base':
        if msg.status[0].message == 'base is not connected or switched off':
            rospy.logerr("Motors are off msg received")
            motor_on = False
        if msg.status[0].message == 'base is present':
            motor_on = True


def reconnect():
        rospy.wait_for_service('/reconnect')
        try:
            recon = rospy.ServiceProxy('/reconnect', Empty)
            recon()
            motor_on = True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    

def launch_driver():
    global motor_on,driver
    print "calling node"
    if not driver is None:
        print driver
        print "killing driver"
    #    driver.kill()
        rospy.sleep(3)
    #driver = subprocess.Popen(['roslaunch','youbot_oodl','youbot_oodl_driver.launch'])
    #rospy.sleep(3)
    motor_on = True

if __name__ == '__main__':

    rospy.init_node('youbot_oodl_watchdog')
    diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, cb_diagnostic)

    #launch_driver()
    #driver = subprocess.Popen(['roslaunch','youbot_oodl','youbot_oodl_driver.launch'])
    #rospy.sleep(10)

    #rate = rospy.Rate(1./4)

    
    while not rospy.is_shutdown():
        if not motor_on:
            rospy.logerr("Motors are off, trying to reconnect")
            reconnect()
#launch_driver();
        rospy.sleep(3)
