#!/usr/bin/env python

import roslib; roslib.load_manifest('slaw_bmt')
import rospy
import smach
import smach_ros
import actionlib
import tf
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from math import pi
from slaw_bnt.bnt import create_goal_message
from slaw_navigation.msg import *
from slaw_manipulation.msg import *

objects = None
positions = None
TIME_OUT = 20.
NORTH = 0.0
MAX_TRIES = 3.
TRIES_FOR_FINE_ADJUST = 300
PRE_GRIP_PLACE = 0.05
AFTER_GRIP_PLACE = 0.15
THRESHOLD_PRE_PLACE = 0.01
THRESHOLD_AFTER_PLACE = 0.03

NO_MOVE_BASE = False
NO_GRIP = False

MAX_TRIES_NOT_FOUND = 0

OFFSET = 0.84 #one plate width + 4 cm profile

TABLE_HEIGHT = 0.045 #for drop

class PlaceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        if NO_GRIP:
            return
        self.client = actionlib.SimpleActionClient('grip_and_place', GripOrPlaceAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        if NO_GRIP:
            return 'success'
       
        goal = GripOrPlaceGoal()
        goal.position = "left"
        goal.grip = False
        goal.height = TABLE_HEIGHT
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))

        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            return 'success'
        else:
            self.client.cancel_all_goals()
            return 'failed'


class GripState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed', 'nothing_found'])
        if NO_GRIP:
            return
        self.client = actionlib.SimpleActionClient('visual_servoing', VisualServoingAction)
        self.client.wait_for_server()

        self.counter = MAX_TRIES_NOT_FOUND
        
    def execute(self, userdata):
        if NO_GRIP:
            return 'success'
        goal = VisualServoingGoal()
        goal.position = "left"
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(2*TIME_OUT))
        #self.client.wait_for_result(rospy.Duration(2))
        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            return 'success'
        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'nothing_found'
            else:
                self.client.cancel_all_goals()
                return 'failed'


        

    
class FineAdjust(smach.State):
    def __init__(self, dist, threshold, preplace, use_offset):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.client = actionlib.SimpleActionClient('fine_adjust', FineAdjustAction)
        self.client.wait_for_server()
        self.dist = dist
        #self.offset = offset
        self.threshold = threshold
        self.preplace = preplace
        self.step_offset = 0.1
        self.counter = 0
        self.use_offset = use_offset
        self.use_initial = True
        
    def execute(self, userdata):
        goal = FineAdjustGoal()
        goal.distance = self.dist
        goal.iterations = TRIES_FOR_FINE_ADJUST
        goal.position = "left"
        goal.use_initial = True
        if self.use_offset:
            goal.use_offset = True
            if self.preplace:
                goal.offset = -4*self.step_offset + self.counter*self.step_offset + OFFSET
                self.counter += 1
            else:
                goal.offset = 0.
        else:
            goal.use_offset = False
            goal.offset = 0.0
        goal.threshold = self.threshold

        
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))
        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            return 'success'
        else:
            self.client.cancel_all_goals()
            return 'failed'

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
 

    
class MoveState(smach.State):
    def __init__(self, goal):
        global positions
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed'])
        rospy.logdebug('Init state with %s'%str(goal))

        if NO_MOVE_BASE:
            return


        ##start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.loginfo("%s"%str(goal))
        rospy.loginfo("%s"%str( positions))
        pose = positions[goal]
        
        pose['theta'] = dir_to_theta(pose['dir'])

        
        self.goal_msg = create_goal_message(pose)
        # todo check if data dictionary holds all relevant data

        rospy.loginfo('Init state "%s"'%self.goal_msg)
        self.counter = MAX_TRIES


    def execute(self, userdata):
        #rospy.loginfo('Executing state "%s"'%self.label)
        if NO_MOVE_BASE:
            return 'reached'
        self.client.send_goal(self.goal_msg)
        self.client.wait_for_result(rospy.Duration(TIME_OUT))

        #rospy.logerr(self.client.get_state())
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goals")
            self.client.cancel_all_goals()
            #rospy.sleep(self.goal['sleep'])
          
            return 'reached'

        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.client.cancel_all_goals()
                return 'failed'


   
class DetectObjectsState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached', 'failed'])
        self.counter = 0

    def execute(self, userdata):
        pass


def load_yaml():
    global objects
    global positions
    
    # load goals from parameter server
    positions = rospy.get_param('slaw_bmt/positions')
    rospy.loginfo('Loaded goal locations from parameter server: %s'%str(positions))

    # bmt plan
    plan = rospy.get_param('slaw_bmt/plan')
    rospy.loginfo('Loaded plan locations from parameter server: %s'%str(plan))

    # bmt plan
    objects = rospy.get_param('slaw_bmt/objects')
    rospy.loginfo('Loaded objects from parameter server: %s'%str(objects))
    
    return plan


def main():
    rospy.init_node('bmt_state_machine')

    plan = load_yaml()
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['failed', 'success'])

    with sm:
        #smach.StateMachine.add("start", MoveState(plan['start']), transitions = {'failed': "move_to_source", 'not_reached': 'start', 'reached':'move_to_source'})
        
        
        smach.StateMachine.add("move_to_source", MoveState(plan['source']), transitions = {'failed': "fine_adjust", 'not_reached': 'move_to_source', 'reached':'fine_adjust'})
            
        #smach.StateMachine.add("detect", DetectObjectsState(), transitions = {'failed': "end", 'detected':'fine_adjust'})

        rospy.loginfo("createing fine adjust")
        smach.StateMachine.add("fine_adjust", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, False, True), transitions = {'failed': "move_to_source", 'success':'grip'})
       
        rospy.loginfo("createing grip")
        
        smach.StateMachine.add("grip", GripState(), transitions = {'failed': 'fine_adjust_after_finally_failed_grip', 'nothing_found': "fine_adjust_after_failed_grip",  'success':'fine_adjust_after_grip'})

        smach.StateMachine.add("fine_adjust_after_failed_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False, False), transitions = {'failed': "move_to_source", 'success':'move_to_source'})

        smach.StateMachine.add("fine_adjust_after_finally_failed_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False, False), transitions = {'failed': "end", 'success':'end'})

        
        smach.StateMachine.add("fine_adjust_after_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False, False), transitions = {'failed': "move_to_dest", 'success':'move_to_dest'})


        smach.StateMachine.add("move_to_dest", MoveState(plan['dest']), transitions = {'failed': "fine_adjust_pre_place", 'not_reached': 'move_to_dest', 'reached':'fine_adjust_pre_place'})

        rospy.loginfo("creating place node")

        smach.StateMachine.add("fine_adjust_pre_place", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, True, True), transitions = {'failed': "place", 'success':'place'})
        
        smach.StateMachine.add("place", PlaceState(), transitions = {'failed': "fine_adjust_after_place", 'success': 'fine_adjust_after_place'})

        smach.StateMachine.add("fine_adjust_after_place", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False, False), transitions = {'failed': "move_to_source", 'success':'move_to_source'})

        smach.StateMachine.add("end", MoveState(plan['end']), transitions = {'failed': 'failed', 'not_reached': 'end', 'reached':'success'})
       

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_BMT')
    sis.start()
    rospy.loginfo("starting!")

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

    
