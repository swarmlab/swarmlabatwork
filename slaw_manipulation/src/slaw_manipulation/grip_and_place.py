#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_manipulation')

import rospy
import tf

import math
import actionlib

import numpy as np

from slaw_arm_navigation.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from actionlib_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from slaw_manipulation.msg import *

from slaw_vision.msg import *

import kinematics_msgs.srv
import kinematics_msgs.msg
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv

import sys
import select

from std_srvs.srv import Empty


pre_grip = [1.4247985844458738, 1.1886094974370305, -1.5520567266162335, 3.354933343438852, 2.9619908989289345]

base_frame = '/arm_link_0'
end_effector_frame = '/arm_link_5'
joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

arm_up_pose = [0.011, 1.04883, -2.43523, 1.73184, 0.2]

WAIT_FOR_SERVERS = 2.0

tf_poses = [[0.02, 0.27, 0.11], [0.02, 0.28, 0.10], [0.02, 0.29,0.085]]
arm_tucked = [0.011, 0.011, -0.016, 0.023, 0.111]

TABLE_HEIGHT = 0.045 #0.045 # TODO change back !!!
STEP_SIZE = 0.05


class GripAndPlace:
    def __init__(self):
        self.gotArmConf = False
        self.configuration = [0,0,0,0,0]
        # Connect to controller state
        rospy.Subscriber('/joint_states', JointState, self.stateCb)

        #Gripper server
        self.gripper_client = actionlib.SimpleActionClient('gripper_action', TuckArmAction)
        if not self.gripper_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVERS)):
            rospy.logerr("gripper action server did not come up within timelimit")

        gripper_action_name = rospy.get_param('~gripper_joint_trajectory_action', '/arm_1/gripper_controller/joint_trajectory_action')
        self.gripper_joint_client = actionlib.SimpleActionClient(gripper_action_name, FollowJointTrajectoryAction)


        #Arm joint action server
        self.move_duration = rospy.get_param('~move_duration', 2.5)
        arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
        self.arm_joint_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
        if not self.arm_joint_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVERS)):
            rospy.logerr("arm_joint_client action server did not come up within timelimit")

        rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
        rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')

        self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
        rospy.loginfo("Service 'get_constraint_aware_ik' is ready")


        rospy.loginfo("Waiting for 'set_planning_scene_diff' service")
        rospy.wait_for_service('/environment_server/set_planning_scene_diff')
        self.planning_scene = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
        rospy.loginfo("Service 'set_planning_scene_diff'")

        # a planning scene must be set before using the constraint-aware ik!
        self.send_planning_scene()
        
        rospy.loginfo('initialized')

        self.action_server = actionlib.simple_action_server.SimpleActionServer('grip_and_place', GripOrPlaceAction, self.executeCB)


    def executeCB(self, goal):
        side = goal.position
        grip = goal.grip
        result = GripOrPlaceResult()
        height = goal.height
        if grip:
            result.result = self.grip()
        else:
            if height == 0:
                result.result = self.place(side = side)
            else:
                result.result = self.place(side = side, height=height)
        if result.result:
            self.action_server.set_succeeded(result)
            rospy.loginfo("done")
        else:
            self.action_server.set_aborted(result)

    def stateCb(self, msg):
        for k in range(5):
            for i in range(len(msg.name)):
                joint_name = "arm_joint_" + str(k + 1)
                if(msg.name[i] == joint_name):
                    self.configuration[k] = msg.position[i]
                    self.gotArmConf = True

        
    def gripperClose(self, close_grip):
        grip = TuckArmGoal()
        grip.tuck_gripper = close_grip
        self.gripper_client.send_goal_and_wait(grip, rospy.Duration(30.0), rospy.Duration(5.0))

    def armUp(self):
        if not self.gotArmConf:
            print 'did not get arm conf yet'
            return
        arm_up_without_turn = [[x for x in arm_up_pose]]
        arm_up_without_turn[0][0] = self.configuration[0]
        arm_up_without_turn[0][4] = self.configuration[4]
             
        return self.go(arm_up_without_turn)

    def armTuck(self):
        if not self.gotArmConf:
            print 'did not get arm conf yet'
            return
        arm_up_without_turn = [[x for x in arm_tucked]]
        arm_up_without_turn[0][0] = self.configuration[0]
        arm_up_without_turn[0][4] = self.configuration[4]
             
        return self.go(arm_up_without_turn)


    def adjustConf(self, conf, position = 'left'):
        if not self.gotArmConf:
            print 'did not get arm conf yet'
            return

        if conf[3] > 3.4292:
            conf[3] = 3.4291
        if position == 'left':
            conf[0] = 1.42
        else:
            conf[0] = 1.42 + math.pi
        conf[4] = self.configuration[4]
        return conf
    
    def go(self, positions, wait = True, gripper = False):
        if self.action_server.is_preempt_requested():
            #rospy.loginfo('%s: Preempted' % self._action_name)
            self.action_server.set_preempted()
            return False
            

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [x for x in joint_names]
        joint_names_gripper = ["gripper_finger_joint_l","gripper_finger_joint_r"]

        if gripper:
            goal.trajectory.joint_names = [x for x in joint_names_gripper]
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                          velocities = [],
                                                          accelerations = [],
                                                          time_from_start = rospy.Duration((count+1) * self.move_duration)))
            goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
            if wait:
                if not gripper:
                    self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
                else:
                    self.gripper_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
                return True
            else:
                if not gripper:
                    self.arm_joint_client.send_goal(goal)
                else:
                    self.gripper_joint_client.send_goal(goal)
        return True

    def send_planning_scene(self):
        rospy.loginfo("Sending planning scene")
        req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
        res = self.planning_scene.call(req)
                
    def call_constraint_aware_ik_solver(self, goal_pose):
        req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
        req.timeout = rospy.Duration(0.5)
        req.ik_request.ik_link_name = "arm_link_5"
        req.ik_request.ik_seed_state.joint_state.name = joint_names
        req.ik_request.ik_seed_state.joint_state.position = self.configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ciks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))
            
        if (resp.error_code.val == arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS):
            return resp.solution.joint_state.position
        else:
            return None
        
    def sendIKGoal(self, pose, quat, onlyReturn = False):
        goal = YoubotIKGoal()
        goal.gripper_pose.pose.position.x = pose[0]
        goal.gripper_pose.pose.position.y = pose[1]
        goal.gripper_pose.pose.position.z = pose[2]

        goal.gripper_pose.header.frame_id = base_frame
        goal.gripper_pose.header.stamp = rospy.Time.now()
        
        goal.gripper_pose.pose.orientation.x = quat[0]
        goal.gripper_pose.pose.orientation.y = quat[1]
        goal.gripper_pose.pose.orientation.z = quat[2]
        goal.gripper_pose.pose.orientation.w = quat[3]
             
        conf = self.call_constraint_aware_ik_solver(goal.gripper_pose)
        print conf
        if onlyReturn:
            if conf:
                return np.array(conf)
            else:
                return None
        else:
            if conf:
                self.go([conf])
                return np.array(conf)
            return conf
        
        
    def pre_grip(self):
        return self.go([pre_grip])
        

    def gripFast(self, grip, height, position = 'left'):
        quat = tf.transformations.quaternion_from_euler(math.pi-0.001,0.001,-math.pi/2+0.001)
        last_pose = [x for x in tf_poses[-1]]
        conf = self.sendIKGoal(tf_poses[0], quat, onlyReturn = True)
        if conf is not None:
            conf[4] = self.configuration[4]
            conf = self.adjustConf(conf, position = position)
            if not self.go([conf]):
                return False
            rospy.sleep(0.5)

        last_pose[2] = height+0.04
        
        conf = self.sendIKGoal(last_pose, quat, onlyReturn = True)
        if conf is not None:
            conf = self.adjustConf(conf, position = position)
            if not self.go([conf]):
                return False
            rospy.sleep(0.5)

        last_pose[2] = height
        conf = self.sendIKGoal(last_pose, quat, onlyReturn = True)
        if conf is not None:
            conf = self.adjustConf(conf, position = position)
            if not self.go([conf]):
                return False
            rospy.sleep(0.5)

        self.gripperClose(grip)
        rospy.sleep(2)

        #grip_conf = [0, 0.0115]
        #self.go([grip_conf], gripper = True)
        #rospy.sleep(1)
        #grip_conf = [0.0115, 0]
        #self.go([grip_conf], gripper = True)
        #rospy.sleep(1)
        self.gripperClose(grip)
          

        conf = self.sendIKGoal(tf_poses[0], quat, onlyReturn = True)
        if conf is not None:
            conf = self.adjustConf(conf, position = position)
            if not self.go([conf]):
                return False

        return True
    
        
    
    def gripRecorded(self, grip):
        quat = tf.transformations.quaternion_from_euler(math.pi-0.001,0.001,-math.pi/2+0.001)
        last_pose = [x for x in tf_poses[-1]]
        for tf_pose in tf_poses:
            conf = self.sendIKGoal(tf_pose, quat, onlyReturn = True)
            if conf is not None:
                conf[4] = self.configuration[4]
                conf = self.adjustConf(conf)
                if not self.go([conf]):
                    return False
                rospy.sleep(0.5)
        while last_pose[2] > TABLE_HEIGHT:
            last_pose[2] -= 0.005
            conf = self.sendIKGoal(last_pose, quat, onlyReturn = True)
            if conf is not None:
                conf = self.adjustConf(conf)
                if not self.go([conf]):
                    return False
                rospy.sleep(0.5)

        self.gripperClose(grip)
        rospy.sleep(2)

        conf = self.sendIKGoal(tf_poses[0], quat, onlyReturn = True)
        if conf is not None:
            conf = self.adjustConf(conf)
            if not self.go([conf]):
                return False
        return True
    

    def grip(self):
        self.gripperClose(False)
        rospy.sleep(1.5)
        return self.gripRecorded(True)
        

    def place(self, side = 'left', height = TABLE_HEIGHT):
        if side == 'right':
            pose = [x for x in self.configuration]
            pose[0] += math.pi
            rospy.loginfo("pos to %f"%pose[0])

            self.go([pose])
            rospy.sleep(2)
        pose = [x for x in self.configuration]
        gripper_pos = 2.96 
        pose[4] = gripper_pos
        rospy.loginfo("rotate gripper_pos to %f"%gripper_pos)
        self.go([pose])
        res =  self.gripFast(False, height, position = side)
        if res:
            if side == 'right':
                self.armTuck()
                pose = [x for x in self.configuration]
                pose[0] -= math.pi
                rospy.loginfo("pos to %f"%pose[0])
                self.go([pose])
                rospy.sleep(2)
                return True
            else:
                return self.armTuck()
        return False
    
def main():
    rospy.init_node("GripAndPlace")
    rospy.sleep(0.001)  # wait for time
    grip_node = GripAndPlace()
    rospy.spin()

if __name__ == '__main__':
    main()

