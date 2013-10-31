#!/usr/bin/env python

PACKAGE = 'slaw_arm_navigation'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
import actionlib
from control_msgs.msg import *
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import *



from slaw_arm_navigation.cfg import ArmConfigConfig


joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
move_duration = 2.5



class ArmConfigServer:

    def __init__(self):
        arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
        self.arm_joint_client = arm_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
        if not self.arm_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("tuck_arms: arm_joint_client action server did not come up within timelimit")
        self.configuration = [0, 0, 0, 0, 0]
        self.gotPose = False

        self.sendCommand = False
        srv = Server(ArmConfigConfig, self.callback)
        # Connect to controller state
        rospy.Subscriber('/joint_states', JointState, self.stateCb)
        self.client = dynamic_reconfigure.client.Client("armConf", timeout=30, config_callback=self.test)

    def test(self, config):
        print "this"

    def updateConf(self):
        pass
        #print "update"
        #self.client.update_configuration({"joint_1": self.configuration[0],"joint_2": self.configuration[1], "joint_3": self.configuration[2], "joint_4": self.configuration[3], "joint_5": self.configuration[4]})
        #rospy.set_param('/armConf',{"joint_1": self.configuration[0],"joint_2": self.configuration[1], "joint_3": self.configuration[2], "joint_4": self.configuration[3], "joint_5": self.configuration[4]})
       
    def stateCb(self, msg):
        for k in range(5):
            for i in range(len(msg.name)):
                joint_name = "arm_joint_" + str(k + 1)
                if(msg.name[i] == joint_name):
                    self.configuration[k] = msg.position[i]
                    self.gotPose = True
    
    def callback(self, config, level):
        if self.sendCommand:
            config.joint_1 = self.configuration[0]
            config.joint_2 = self.configuration[1]
            config.joint_3 = self.configuration[2]
            config.joint_4 = self.configuration[3]
            config.joint_5 = self.configuration[4]

            return config

            
        #r = rospy.Rate(10)
        #while not self.gotPose:
        #    r.sleep()

            
    #rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\
    #      {str_param}, {bool_param}, {size}""".format(**config))
        conf = [config.joint_1]
        conf.append( config.joint_2)
        conf.append( config.joint_3)
        conf.append( config.joint_4)
        conf.append( config.joint_5)

        print conf
        self.sendCommand = True
        self.go([conf])

        config.joint_1 = self.configuration[0]
        config.joint_2 = self.configuration[1]
        config.joint_3 = self.configuration[2]
        config.joint_4 = self.configuration[3]
        config.joint_5 = self.configuration[4]

        return config



    def go(self, positions, wait = True):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [x for x in joint_names]
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                                velocities = [],
                                                                accelerations = [],
                                                                time_from_start = rospy.Duration((count+1) * move_duration)))
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
  
        if wait:
            self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            self.sendCommand = False
        else:
            self.arm_joint_client.send_goal(goal)



if __name__ == "__main__":
        rospy.init_node("armConf")
        server = ArmConfigServer()

        r = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            server.updateConf()
            r.sleep()
