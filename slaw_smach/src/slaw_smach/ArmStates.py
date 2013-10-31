#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_smach')

import rospy
import tf

import math
import actionlib

import smach

import numpy as np

from slaw_arm_navigation.msg import *
from slaw_arm_navigation.srv import *

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

from actionlib_msgs.msg import *
from sensor_msgs.msg import JointState

from geometry_msgs.msg import *

#from slaw_vision.msg import *

pre_kinect = [5.323699090192345, 1.3339887113233428, -2.446201119717692, 1.8014157762386995, 2.9404422281923392]

#pre_kinect = [5.323699090192345, 1.3339887113233428, -2.54823, 1.7885084, 2.923]

pre_grip = [5.323699090192345, 1.134883, -4.119026326794897, 3.3593047267948966, 2.923]

base_frame = '/arm_base_link'
end_effector_frame = '/arm_tip_link'

joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

joints = {}

grip_point = [0.21, 0.0, 0.0] #0.28 #0.15 

PRE_GRIP_HEIGHT = 0.04
PLACE_OFFSET = 0.1


TABLE_HEIGHT = 0.102
ABOVE_TABLE = 0.005


MAX_TRIES = None
TIME_OUT = None
NO_ARM = None
arm_client = None
gripper_client = None
iks = None
mount_offset = None
arm_base_link_height = None


configuration = [0, 0, 0, 0, 0]
gotArmConf = False


def init_globals():
    global MAX_TRIES, TIME_OUT, NO_ARM, arm_client, gripper_client, iks, joints, mount_offset, arm_base_link_height, grip_point
    if MAX_TRIES is None:
        MAX_TRIES = rospy.get_param("smach/max_tries", 5)
        TIME_OUT = rospy.get_param("smach/time_out", 5.0)
        NO_ARM = rospy.get_param("smach/no_arm", False)
        
        rospy.Subscriber('/joint_states', JointState, joint_state_cb)

        joints = rospy.get_param("joints")
        mount_offset = rospy.get_param("arm_rot_offset")
        #Gripper client
        gripper_client = actionlib.SimpleActionClient('gripper_action', TuckArmAction)
        arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
        arm_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
        iks = rospy.ServiceProxy('/arm_1/simple_ik_server', SimpleIkSolver)

        arm_base_link_height = rospy.get_param('arm_base_link_height')

        grip_point[2] = TABLE_HEIGHT + ABOVE_TABLE - arm_base_link_height

        if not NO_ARM:
            if not gripper_client.wait_for_server(rospy.Duration(TIME_OUT)):
                rospy.logerr("gripper action server did not come up within timelimit")
            if not arm_client.wait_for_server(rospy.Duration(TIME_OUT)):
                rospy.logerr("arm_joint_client action server did not come up within timelimit")
            
        rospy.loginfo("globals initialized")
        rospy.sleep(0.5)
    else:
        rospy.loginfo("globals already initialized")


def call_ik_solver(goal_point, side = 'front'):
        req = SimpleIkSolverRequest()
        req.position = side
        
        req.point = PointStamped()
        req.point.point.x = goal_point[0]
        req.point.point.y = goal_point[1]
        req.point.point.z = goal_point[2]

        req.point.header.frame_id = base_frame

        req.point.header.stamp = rospy.Time()
        resp = None
        try:
            resp = iks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))

        return resp.joints


def joint_state_cb(msg):
    global configuration, gotArmConf
    for k in range(5):
        for i in range(len(msg.name)):
            joint_name = "arm_joint_" + str(k + 1)
            if(msg.name[i] == joint_name):
                configuration[k] = msg.position[i]
                gotArmConf = True

                
def rotate_only(side, turned = False):
    rotate_only = [x for x in configuration]
    straight = joints['arm_joint_1']['straight'] - mount_offset
    
    rotate_only[0] = straight

    if (side == 'left'):
        rotate_only[0] += -math.pi/2
    elif (side == 'right'):
        rotate_only[0] += +math.pi/2
    elif (side == 'back'):
        rotate_only[0] += math.pi
    
    if turned:
        rotate_only[0] = limit_joint1_ang(rotate_only[0]+math.pi)
    return rotate_only


def limit_joint1_ang(ang):
    while ang < joints['arm_joint_1']['min']:
        ang = ang + 2*math.pi
    while ang > joints['arm_joint_1']['max']:
        ang = ang - 2* math.pi
    return ang
    


def rotate_only_angle(angle):
    rotate_only = [x for x in configuration]
    rotate_only[0] = angle
    return rotate_only


def gripperClose(close_grip):
    grip = TuckArmGoal()
    grip.tuck_gripper = close_grip
    gripper_client.send_goal_and_wait(grip, rospy.Duration(30.0), rospy.Duration(5.0))


def go(positions, move_duration = 2.5):
    if NO_ARM:
        rospy.sleep(1.0)
        return True
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [x for x in joint_names]
    goal.trajectory.points = []
    for p, count in zip(positions, range(0,len(positions)+1)):
        goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                            velocities = [],
                                                            accelerations = [],
                                                            time_from_start = rospy.Duration((count+1) * move_duration)))
        
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

    arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
    res = arm_client.get_result()

    if arm_client.get_state() == GoalStatus.SUCCEEDED:
        return True

    rospy.logerr("Arm failed!!!")
    return False


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['offset_in', 'pose_in'], output_keys=['pose_out'])
        init_globals()
        self.counter = {}
                
    def execute(self, userdata):
        #offset detected by object detection
        #offset = userdata.offset_in

        #position where we are
        pose = userdata.pose_in
        locations = rospy.get_param('locations')
        userdata.pose_out = locations[0]['name']
        

        position = rospy.get_param(pose)
        #to which side
        side = position['side']

        #quat = [offset.orientation.x, offset.orientation.y, offset.orientation.z,offset.orientation.w]
        #r, p, y = tf.transformations.euler_from_quaternion(quat)

        if pose not in self.counter.keys():
            self.counter[pose] = 0

        if self.counter[pose] >= 4:
            self.counter[pose] = 0
        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += 0.05
        new_grip_point[1] = -0.15
        new_grip_point[1] += self.counter[pose] * PLACE_OFFSET

        
        conf = np.array(call_ik_solver(new_grip_point, side = side))
        
        if conf is None:
            userdata.offset_out = new_grip_point[1]
            return 'too_far'

        new_grip_point[2]+= PRE_GRIP_HEIGHT

        conf2 = None

        while conf2 is None:
            conf2 = np.array(call_ik_solver(new_grip_point, side = side))
            new_grip_point[0]-= 0.01 #get closer to solve pre-grip
            

        rotate = rotate_only(side)

        if not go([rotate, conf2, conf]):
            return 'failed'

        gripperClose(False)
        
        tucked = rotate_only(side)
        tucked[0] = conf[0]
        tucked[1] = joints['arm_joint_2']['min']
        tucked[2] = joints['arm_joint_3']['max']
        tucked[3] = joints['arm_joint_4']['min']
        tucked[4] = conf[4]

        tucked_for_drive = rotate_only(side = 'left')
        #tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']

        #obj = rospy.get_param(userdata.object_in)
        #
        #userdata.object_out = userdata.object_in

        go([conf2, tucked, tucked_for_drive])
        #if not go([conf2, tucked, tucked_for_drive]):
        #    return 'failed_after_place'
        self.counter[pose] += 1

        return 'success'

class FinePlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'too_far', 'failed', 'failed_after_place'], input_keys=['pose_in', 'point_in', 'object_in'], output_keys=['pose_out'])
        init_globals()


    def getAngle(self, straight, angle):
        ang = straight - angle
        #print straight, angle
        while ang < straight-math.pi/2:
            ang = ang + math.pi
        while ang > straight+math.pi/2:
            ang = ang - math.pi
        return ang

    
    def execute(self, userdata):
        #offset detected by object detection
        offset = userdata.point_in
        
        pose = userdata.pose_in
        userdata.pose_out = pose

        obj = userdata.object_in 
        position = rospy.get_param(pose)
        #to which side
        side = position['side']

        quat = [offset.orientation.x, offset.orientation.y, offset.orientation.z,offset.orientation.w]
        r, p, y = tf.transformations.euler_from_quaternion(quat)


        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += offset.position.x
        new_grip_point[1] -= offset.position.y
        
        conf = np.array(call_ik_solver(new_grip_point, side = side))
        
        if conf is None:
            return 'too_far'

        conf[4] = self.getAngle(conf[4], y)
        new_grip_point[2]+= PRE_GRIP_HEIGHT



        rotate = rotate_only(side = side)

        height = grip_point[2]        

        gripX = new_grip_point[0]
        gripY = new_grip_point[1]
        
        new_grip_point[0] = gripX
        new_grip_point[1] = gripY
        conf2 = None

        while conf2 is None:
            conf2 = np.array(call_ik_solver(new_grip_point, side = side))
            new_grip_point[0]-= 0.005 #get closer to solve pre-grip
            new_grip_point[1]-= 0.005 #get closer to solve pre-grip
        conf2[4] = conf[4]
        
            
        #conf2[3] += 0.1# adjust for overshoot!

        if not go([rotate, conf2, conf]):
            return 'failed'
        
        gripperClose(False)

        #rotate??
        if False:
            rotate_wiggle = [x for x in conf]
            rotate_wiggle[4] -= 0.1
        
            rotate_wiggle2 = [x for x in conf]
            rotate_wiggle2[4] += 0.1

            if not go([rotate_wiggle, rotate_wiggle2, conf]):
                return 'failed'

        if obj in ['V20', 'R20']:
            go([conf2])
            gripperClose(True)
            go([conf])
        
        tucked = rotate_only(side)
        tucked[0] = conf[0]
        tucked[1] = joints['arm_joint_2']['min']
        tucked[2] = joints['arm_joint_3']['max']
        tucked[3] = joints['arm_joint_4']['min']
        tucked[4] = conf[4]

        tucked_for_drive = rotate_only(side = 'left')
        #tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']

        locations = rospy.get_param('locations')
        userdata.pose_out = locations[0]['name']

        if not go([conf2, tucked, tucked_for_drive]):
            return 'failed_after_place'
        return 'success'


    


class Grip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'too_far', 'failed', 'failed_after_grip'], input_keys=['pose_in', 'point_in', 'object_in'], output_keys=['pose_out', 'point_out', 'object_out'])
        init_globals()


    def getAngle(self, straight, angle):
        ang = straight - angle
        #print straight, angle
        while ang < straight-math.pi/2:
            ang = ang + math.pi
        while ang > straight+math.pi/2:
            ang = ang - math.pi
        return ang

    
    def execute(self, userdata):
        #offset detected by object detection
        offset = userdata.point_in
        
        pose = userdata.pose_in
        userdata.pose_out = pose


        position = rospy.get_param(pose)
        #to which side
        side = position['side']

        quat = [offset.orientation.x, offset.orientation.y, offset.orientation.z,offset.orientation.w]
        r, p, y = tf.transformations.euler_from_quaternion(quat)


        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += offset.position.x
        new_grip_point[1] -= offset.position.y
        
        conf = np.array(call_ik_solver(new_grip_point, side = side))
        
        if conf is None:
            return 'too_far'

        conf[4] = self.getAngle(conf[4], y)
        new_grip_point[2]+= PRE_GRIP_HEIGHT


        confs = []
        rotate = rotate_only(side = side)
        height = grip_point[2]        
        confs.append(rotate)
        first = None
        gripX = new_grip_point[0]
        gripY = new_grip_point[1]
        
        while new_grip_point[2] > height:

            new_grip_point[2] -= 0.004
            new_grip_point[0] = gripX
            new_grip_point[1] = gripY
            conf2 = None

            while conf2 is None:
                conf2 = np.array(call_ik_solver(new_grip_point, side = side))
                new_grip_point[0]-= 0.005 #get closer to solve pre-grip
                new_grip_point[1]-= 0.005 #get closer to solve pre-grip
            conf2[4] = conf[4]
            if first is None:
                first = [x for x in conf2]
            confs.append(conf2)

            
            
        #conf2[3] += 0.1# adjust for overshoot!


        gripperClose(False)
        confs.append(conf)

        if not go(confs):
            return 'failed'
        
        gripperClose(True)


        tucked = rotate_only(side)
        tucked[0] = conf[0]
        tucked[1] = joints['arm_joint_2']['min']
        tucked[2] = joints['arm_joint_3']['max']
        tucked[3] = joints['arm_joint_4']['min']
        tucked[4] = conf[4]

        tucked_for_drive = rotate_only(side = 'left')
        #tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']

        obj = rospy.get_param(userdata.object_in)

        print obj
        userdata.pose_out = obj['place_location']
        userdata.object_out = userdata.object_in
        
        if not go([first, tucked, tucked_for_drive]):
            return 'failed_after_grip'
        return 'success'

class GripCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end'])
        init_globals()
        
    def execute(self, userdata):
        rospy.sleep(4)
        gripperClose(True)
        tucked = [x for x in configuration]
        #tucked[0] = configuration[0]
        tucked[1] = joints['arm_joint_2']['min']
        tucked[2] = joints['arm_joint_3']['max']
        tucked[3] = joints['arm_joint_4']['min']
        #tucked[4] = configuration[4]

        tucked_for_drive = rotate_only(side = 'left')
        #tucked[0] = conf[0]
        tucked_for_drive[1] = joints['arm_joint_2']['min']
        tucked_for_drive[2] = joints['arm_joint_3']['max']
        tucked_for_drive[3] = joints['arm_joint_4']['min']
        tucked_for_drive[4] = joints['arm_joint_5']['straight']

        go([tucked, tucked_for_drive])
  
        return 'end'
    

class PreGripCBT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_globals()

    def getAngle(self, straight, angle):
        ang = straight - angle
        #print straight, angle
        while ang < straight-math.pi/2:
            ang = ang + math.pi
        while ang > straight+math.pi/2:
            ang = ang - math.pi
        return ang

                
    def execute(self, userdata):
        pose = userdata.pose_in
        position = rospy.get_param(pose)
        userdata.pose_out = pose

        side = position['side']
        rotate = rotate_only(side)
        #pre_grip[0] = rotate[0]

        gripperClose(False)
        new_grip_point = [x for x in grip_point]
        new_grip_point[0] += 0.05
        new_grip_point[2] += 0.01

        conf = np.array(call_ik_solver(new_grip_point, side = side))
                
        
        conf[4] = self.getAngle(conf[4], math.pi/2)
    
        if go([rotate, conf]):
            return 'success'
        else:
            return 'failed'


    
    
class PreGrip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_globals()
                
    def execute(self, userdata):
        pose = userdata.pose_in
        position = rospy.get_param(pose)
        
        side = position['side']
        rotate = rotate_only(side, turned = True)
        pre_grip[0] = rotate[0]

        userdata.pose_out = pose
        if go([rotate, pre_grip]):
            return 'success'
        else:
            return 'failed'


        
class TuckArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'not_reached', 'failed'])
        init_globals()
        self.counter = 0

    def execute(self,userdata):

        #side = position['side']
        rotate = rotate_only('left', turned = False)
        arm_up = [rotate[0], joints['arm_joint_2']['straight'], joints['arm_joint_3']['straight'], joints['arm_joint_4']['straight'], rotate[4]] 
        
        tuck = [rotate[0], joints['arm_joint_2']['min'], joints['arm_joint_3']['max'], joints['arm_joint_4']['min'], rotate[4]] 

        #print arm_up, tuck
        #return 'success'
        if go([rotate, arm_up, tuck]):
            self.counter = 0
            return 'success'
        else:
            if self.counter < MAX_TRIES:
                self.counter += 1
                return 'not_reached'
            else:
                self.counter = 0
                return 'failed'

        

        
class PreKinect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_globals()

        
                
    def execute(self, userdata):

        pose = userdata.pose_in
        position = rospy.get_param(pose)
        side = position['side']
        rotate = rotate_only(side, turned = True)
        pre_kinect[0] = rotate[0]
        userdata.pose_out = pose
        if go([rotate, pre_kinect]):
            return 'success'
        else:
            return 'failed'



