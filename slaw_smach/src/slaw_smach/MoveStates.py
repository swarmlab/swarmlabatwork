#!/usr/bin/env python

import roslib; roslib.load_manifest('slaw_smach')
import rospy
import smach
import smach_ros
import actionlib
import tf
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from slaw_location_service.srv import *
from slaw_navigation.srv import *
from slaw_registration.msg import *
from slaw_navigation.msg import *

from math import pi

NORTH = 0.0

MAX_TRIES = None
TIME_OUT = None
NO_MOVE_BASE = None
NO_FINE_ADJUST = None


def init_globals():
    global MAX_TRIES, TIME_OUT, NO_MOVE_BASE, NO_FINE_ADJUST
    if MAX_TRIES is None:
        MAX_TRIES = rospy.get_param("smach/max_tries", 5)
        TIME_OUT = rospy.get_param("smach/time_out", 5.0)
        NO_MOVE_BASE = rospy.get_param("smach/no_move_base", False)
        NO_FINE_ADJUST = rospy.get_param("smach/no_fine_adjust", False)

    
def create_goal_message(goal):
    goal_msg = MoveBaseGoal()

    
    #goal_msg.target_pose.pose.position.x = goal["x"]
    #goal_msg.target_pose.pose.position.y = goal["y"]
    #if 'theta' in goal.keys():
    #    q = tf.transformations.quaternion_from_euler(0,0, goal["theta"], axes='sxyz')
  #      goal_msg.target_pose.pose.orientation.x = q[0]
   #     goal_msg.target_pose.pose.orientation.y = q[1]
   #     goal_msg.target_pose.pose.orientation.z = q[2]
    #    goal_msg.target_pose.pose.orientation.w = q[3]
    #else:
     #   rospy.logwarn("no theta set, defaulting to 0")
     #   goal_msg.target_pose.pose.orientation.w = 1.0
    goal_msg.target_pose.pose = goal

    goal_msg.target_pose.header.frame_id = "/map"
    goal_msg.target_pose.header.stamp = rospy.Time.now()
    return goal_msg


def dir_to_quaternion(direction):
    theta = dir_to_theta(direction)
    q = tf.transformations.quaternion_from_euler(0,0, theta, axes='sxyz')
    return q


def dir_to_theta(direction):
    theta = None
    if direction == 'N':
        theta = .0 + NORTH
    elif direction == 'W':
        theta = .5 * pi + NORTH
    elif direction == 'S':
        theta = pi + NORTH
    elif direction == 'E':
        theta = 1.5 * pi + NORTH
    else:
        rospy.logerr('Direction "%s" is unknown'%(direction))
    return theta


def get_next_goal_bnt(prev_pose):
    #get_goal pose for current object
    locations = rospy.get_param('locations')
    if len(locations) > 0:
        return locations[0]['name']
    else:
        return None
    
def get_dir_bnt(pose):
    locations = rospy.get_param('locations')
    if len(locations) > 0:
        return locations[0]['dir']
    return ""
    
    
    
def get_suffix_bnt():
    locations = rospy.get_param('locations')
    if len(locations) > 0:
        return "".join(["_", locations[0]['dir']])
    return ""


def get_sleep_bnt():
    locations = rospy.get_param('locations')
    if len(locations) > 0:
        return locations[0]['sleep']
    return 0.0

def get_orientation_bnt():
    locations = rospy.get_param('locations')
    if len(locations) > 0:
        return locations[0]['dir']
    return "N"


                       
def get_next_goal_gripping(prev_pose):
    #get source location with most objects
    locations = rospy.get_param('locations')
    best = None
    max_num = -1
    for loc in locations:
        objects = rospy.get_param(loc['name'])
        if len(objects) > max_num:
            max_num = len(objects)
            best = loc
    return best

def get_suffix_grip():
    return "_grip"

def get_orientation_pre_defined(pose):
    loc = rospy.get_param(pose)
    return loc['align']


class RecoverState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'], output_keys=['pose_out'])
        self.switch_off = rospy.ServiceProxy('/scan_registration/switchOffForcefield', switchOff)
        
    
    def sendPause(self, pause):
        rospy.wait_for_service('/scan_registration/switchOffForcefield')
        try:
            resp1 = self.switch_off(pause)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def execute(self, userdata):
        pose = userdata.pose_in
        userdata.pose_out = pose

        self.sendPause(False)
        rospy.sleep(2.0)
        self.sendPause(True)

        return 'done'
        
        
    
class SleepState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'], output_keys=['pose_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)
        pose = userdata.pose_in
        userdata.pose_out = pose
        #locations = rospy.get_param('locations')
        #locations.pop(0)
        #rospy.set_param('locations', locations)
        sleep = get_sleep_bnt()
        rospy.sleep(sleep)
        return 'done'
    

class DelReachedGoalStateBTT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_end', 'move_out'], input_keys=['pose_in'], output_keys=['pose_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)
        pose = userdata.pose_in
        locations = rospy.get_param('locations')
        locations.pop(0)
       
        rospy.set_param('locations', locations)
        if len(locations) > 0:
            userdata.pose_out = locations[0]['name']
            return 'not_end'
        else:
            userdata.pose_out = 'D0'
            return 'move_out'


    
class DelReachedGoalState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'], output_keys=['pose_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)
        pose = userdata.pose_in
        userdata.pose_out = ""
        locations = rospy.get_param('locations')
        locations.pop(0)
        rospy.set_param('locations', locations)
        return 'done'
        

class MoveStateUserData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed'], input_keys=['pose_in'], output_keys=['pose_out'])

        init_globals()
        ##start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        
        if not NO_MOVE_BASE:
            self.client.wait_for_server()

        self.locService = rospy.ServiceProxy("location_service/get_location", GetLocation)

        self.counter = MAX_TRIES
#        self.old_state = None

    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)

        pose = userdata.pose_in
        
        userdata.pose_out = pose
        #Get Goal!!
        rospy.wait_for_service('location_service/get_location')

       
        try:
            goal = self.locService(pose)  #string name
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'failed'

        direction = get_orientation_pre_defined(pose)
        q = dir_to_quaternion(direction)
        
        goal_msg = create_goal_message(goal.locations[0].pose)

        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]
        
        if NO_MOVE_BASE:
            rospy.sleep(2)
            self.counter = MAX_TRIES
            return 'reached'

        self.client.send_goal(goal_msg)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))

        #rospy.logerr(self.client.get_state())
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goals")
            self.client.cancel_all_goals()
            self.counter = MAX_TRIES

            return 'reached'

        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.counter = MAX_TRIES
                return 'failed'


class MoveStateSmart(smach.State):
    def __init__(self, get_next_goal, get_dir, get_suffix):
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed', 'end'], input_keys=['pose_in'], output_keys=['pose_out', 'suffix_out'])
        init_globals()
        ##start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.get_next_goal = get_next_goal
        self.get_suffix = get_suffix
        self.get_dir = get_dir
        
        if not NO_MOVE_BASE:
            self.client.wait_for_server()

        self.locService = rospy.ServiceProxy("location_service/get_location", GetLocation)
        self.counter = MAX_TRIES

    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)
        pose = self.get_next_goal(userdata.pose_in)
        suffix = self.get_suffix()
        
        if pose is None:
            return 'end'
        
        userdata.suffix_out = suffix
        userdata.pose_out = pose
        rospy.wait_for_service('location_service/get_location')

       
        try:
            goal = self.locService(pose)  #string name
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'failed'
        
        direction = self.get_dir(pose)
        q = dir_to_quaternion(direction)
        
        goal_msg = create_goal_message(goal.locations[0].pose)

        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]

        if NO_MOVE_BASE:
            rospy.sleep(2)
            
            #if self.counter > 0:
            #    self.counter -= 1
            #    self.client.cancel_all_goals()
            #    return 'not_reached'
            #else:
            #    self.counter = MAX_TRIES
            #    return 'failed'

            return 'reached'

        self.client.send_goal(goal_msg)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))
        
        #rospy.logerr(self.client.get_state())
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goals")
            self.client.cancel_all_goals()
            #rospy.sleep(sleep)
            self.counter = MAX_TRIES
            return 'reached'

        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.counter = MAX_TRIES
                return 'failed'

            


class MoveBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'], input_keys=['pose_in'], output_keys=['pose_out'])
        init_globals()
  
        self.odom_fine_client = actionlib.SimpleActionClient('odom_move', OdomFineAdjustAction)
  
        if not NO_FINE_ADJUST:
            self.odom_fine_client.wait_for_server()
         
    def execute(self, userdata):
        pose = userdata.pose_in
        userdata.pose_out = pose
        param_pose = rospy.get_param(pose)
        side = param_pose['side']
        
        goal = OdomFineAdjustGoal()
        if side == "left":
            goal.target.x = -0.075
        else:
            goal.target.y = -0.075

        goal.max_dist = 0.2
        goal.duration = 5


        print goal
        if NO_FINE_ADJUST:
            rospy.sleep(2)
            #return 'failed'
            return 'done'

        self.odom_fine_client.send_goal_and_wait(goal)
        return 'done'

        
class ScanMatcher(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','not_reached','failed'], input_keys=['pose_in', 'suffix_in'], output_keys=['pose_out'])

        init_globals()
        self.client = actionlib.SimpleActionClient('registration_fine_adjust', RegistrationFineAdjustAction)

        self.locService = rospy.ServiceProxy("location_service/get_location", GetLocation)
        if not NO_FINE_ADJUST:
            self.client.wait_for_server()
        self.counter = MAX_TRIES

            
    def execute(self, userdata):

        suffix = userdata.suffix_in
        pose = userdata.pose_in

        userdata.pose_out = pose
        try:
            poseLoc = self.locService(''.join([pose, suffix]))  #string name
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'failed'

        if len(poseLoc.locations) > 0:
            goal = RegistrationFineAdjustGoal()
            goal.front = poseLoc.locations[0].scan_front
            goal.rear = poseLoc.locations[0].scan_rear
            goal.x_thresh = 0.01
            goal.y_thresh = 0.01
            goal.theta_thresh = 0.01
            goal.duration = TIME_OUT
        else:
            print "No Location recorded at that pose"
            return 'failed'
            

        if NO_FINE_ADJUST:
            
            rospy.sleep(2)
            #if self.counter > 0:
            #    self.counter -= 1
            #    self.client.cancel_all_goals()
            #    return 'not_reached'
            #else:
            #    self.counter = MAX_TRIES
            #    return 'failed'

            #return 'failed'
            return 'reached'
        
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))

        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            self.counter = MAX_TRIES
            return 'reached'

        else:
            #TODO check if stil "Close"
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.counter = MAX_TRIES
                return 'failed'
