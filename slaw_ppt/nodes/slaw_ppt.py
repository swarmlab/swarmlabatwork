#!/usr/bin/env python

import roslib; roslib.load_manifest('slaw_ppt')
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
PRE_GRIP_PLACE = 0.03
AFTER_GRIP_PLACE = 0.15
THRESHOLD_PRE_PLACE = 0.005
THRESHOLD_AFTER_PLACE = 0.03

NO_MOVE_BASE = False
NO_GRIP = False

MAX_TRIES_NOT_FOUND = 0

X_OFFSET = 0.05

TABLE_HEIGHT = 0.055 #for drop

cur_move_base_goal = None

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



class ScanState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        if NO_GRIP:
            return
        self.client = actionlib.SimpleActionClient('visual_servoing', VisualServoingAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        if NO_GRIP:
            return 'success'
        goal = VisualServoingGoal()
        goal.position = "left"
        goal.only_scanning = True
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(2*TIME_OUT))
        #self.client.wait_for_result(rospy.Duration(2))
        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))


        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            return 'success'
            
        else:
            self.client.cancel_all_goals()
            return 'success'


class GripState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success_long', 'success_short', 'failed', 'nothing_found'])
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
        goal.only_scanning = False

        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(2*TIME_OUT))
        #self.client.wait_for_result(rospy.Duration(2))
        result = self.client.get_result()
        rospy.loginfo("%s"%str(result))


        if self.client.get_state() == GoalStatus.SUCCEEDED:
            self.client.cancel_all_goals()
            if result.object == 'long':
                return 'success_long'
            elif result.object == 'short':
                return 'success_short'
            
        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'nothing_found'
            else:
                self.client.cancel_all_goals()
                return 'failed'


        

    
class FineAdjust(smach.State):
    def __init__(self, dist, threshold, pre_place, dest = None):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.client = actionlib.SimpleActionClient('fine_adjust', FineAdjustAction)
        self.client.wait_for_server()
        self.dist = dist
        #self.offset = offset
        self.threshold = threshold
        self.step_offset = -0.02
        self.counter = 0
        self.pre_place = pre_place
        self.dest = dest
        
    def execute(self, userdata):
        goal = FineAdjustGoal()
        goal.distance = self.dist
        goal.iterations = TRIES_FOR_FINE_ADJUST
        goal.position = "left"
        #NOT USED
        goal.use_initial = True
        goal.front = True
        goal.use_offset = True
        if self.dest is None:
            goal.location  = cur_move_base_goal
        else:
            goal.location  = self.dest
        if self.pre_place:
            if cur_move_base_goal == 'S1':
                goal.offset = self.counter*-self.step_offset
            else:
                goal.offset = self.counter*self.step_offset
            self.counter += 1
        else:
            goal.offset = 0.

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
        self.goal = goal
       
        ##start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not NO_MOVE_BASE:
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
        global cur_move_base_goal
        cur_move_base_goal = self.goal
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




class MoveStateSource(smach.State):
        
    def __init__(self, goals):
        smach.State.__init__(self, outcomes=['reached', 'not_reached', 'failed', 'end_reached', 'end_failed'])
        #rospy.logdebug('Init state with %s'%str(goal))
        self.goals = goals

        rospy.logerr("%s"%goals)
        self.counter = MAX_TRIES
        self.num_goal = 0

        ##start movebasec client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if NO_MOVE_BASE:
            return
        self.client.wait_for_server()


    def execute(self, userdata):
        global positions

        global cur_move_base_goal
 #rospy.loginfo('Executing state "%s"'%self.label)
        goal = self.goals[self.num_goal]
        pose = positions[goal]

        cur_move_base_goal = goal

        pose['theta'] = dir_to_theta(pose['dir'])

        
        self.goal_msg = create_goal_message(pose)
        # todo check if data dictionary holds all relevant data

        rospy.loginfo('Init state "%s"'%self.goal_msg)

        if not NO_MOVE_BASE:
            self.client.send_goal(self.goal_msg)
            self.client.wait_for_result(rospy.Duration(TIME_OUT))

       

        #rospy.logerr(self.client.get_state())
        if NO_MOVE_BASE or self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reached goals")
            self.client.cancel_all_goals()
            #rospy.sleep(self.goal['sleep'])
            self.num_goal += 1
            self.counter = MAX_TRIES
            if self.num_goal == len(self.goals):
                return 'end_reached'
            else:
                return 'reached'

        else:
            if self.counter > 0:
                self.counter -= 1
                self.client.cancel_all_goals()
                return 'not_reached'
            else:
                self.num_goal += 1
                self.counter = MAX_TRIES
                self.client.cancel_all_goals()
                if self.num_goal == len(self.goals):
                    return 'end_failed'
                else:
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
    positions = rospy.get_param('/positions')
    rospy.loginfo('Loaded goal locations from parameter server: %s'%str(positions))

    # bmt plan
    plan = rospy.get_param('slaw_ppt/plan')
    rospy.loginfo('Loaded plan locations from parameter server: %s'%str(plan))

    # bmt plan
#    objects = rospy.get_param('slaw_ppt/objects')
 #   rospy.loginfo('Loaded objects from parameter server: %s'%str(objects))
    
    return plan


def createGoalListWithNumCount(plan):
    goals = []
    rospy.logerr("%s"%plan['sources'])
    for loc in plan['sources']:
        rospy.logerr("%s"%loc)
        for i in xrange(loc['num_objects']):
            goals.append(loc['label'])
    goals.append(plan['end'])
    return goals
                  


def main():
    rospy.init_node('ppt_state_machine')

    plan = load_yaml()
    goals = createGoalListWithNumCount(plan)
    # create a SMACH state machine
    sm = smach.StateMachine(outcomes=['failed', 'success'])

    
    with sm:
        #smach.StateMachine.add("start", MoveState(plan['start']), transitions = {'failed': "move_to_source", 'not_reached': 'start', 'reached':'move_to_source'})

                #fineadjust, align dist, threshold (precision), preplace, use_offset (use x), use_initial, dest):

        smach.StateMachine.add("move_to_dest_scan", MoveState('S5'), transitions = {'failed': "fine_adjust_pre_scan", 'not_reached': 'move_to_dest_scan', 'reached':'fine_adjust_pre_scan'})
        smach.StateMachine.add("fine_adjust_pre_scan", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, False), transitions = {'failed': "scan", 'success':'scan'})
        smach.StateMachine.add("scan", ScanState(), transitions = {'success':'fine_adjust_after_scan'})
        smach.StateMachine.add("fine_adjust_after_scan", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False), transitions = {'failed': "move_to_source", 'success':'move_to_source'})




        smach.StateMachine.add("move_to_source", MoveStateSource(goals), transitions = {'failed': "fine_adjust", 'not_reached': 'move_to_source', 'reached':'fine_adjust', 'end_reached': 'success', 'end_failed': 'failed'})

        smach.StateMachine.add("fine_adjust", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, False), transitions = {'failed': "move_to_source", 'success':'grip'})


        smach.StateMachine.add("grip", GripState(), transitions = {'failed': 'fine_adjust_after_failed_grip', 'nothing_found': "fine_adjust_after_failed_grip", 'success_long':'fine_adjust_after_long_grip', 'success_short': 'fine_adjust_after_short_grip'})

        smach.StateMachine.add("fine_adjust_after_failed_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False), transitions = {'failed': "move_to_source", 'success':'move_to_source'})

        
        smach.StateMachine.add("fine_adjust_after_long_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False), transitions = {'failed': "move_to_dest_long", 'success':'move_to_dest_long'})
        smach.StateMachine.add("move_to_dest_long", MoveState(plan['destLong']), transitions = {'failed': "fine_adjust_pre_place_long", 'not_reached': 'move_to_dest_long', 'reached':'fine_adjust_pre_place_long'})
        smach.StateMachine.add("fine_adjust_pre_place_long", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, True, dest = plan['destLong']), transitions = {'failed': "place", 'success':'place'})       


        smach.StateMachine.add("fine_adjust_after_short_grip", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False), transitions = {'failed': "move_to_dest_short", 'success':'move_to_dest_short'})
        smach.StateMachine.add("move_to_dest_short", MoveState(plan['destShort']), transitions = {'failed': "fine_adjust_pre_place_short", 'not_reached': 'move_to_dest_short', 'reached':'fine_adjust_pre_place_short'})
        smach.StateMachine.add("fine_adjust_pre_place_short", FineAdjust(PRE_GRIP_PLACE, THRESHOLD_PRE_PLACE, True, dest = plan['destShort']), transitions = {'failed': "place", 'success':'place'})       
        
        smach.StateMachine.add("place", PlaceState(), transitions = {'failed': "fine_adjust_after_place", 'success': 'fine_adjust_after_place'})
        
        smach.StateMachine.add("fine_adjust_after_place", FineAdjust(AFTER_GRIP_PLACE, THRESHOLD_AFTER_PLACE, False), transitions = {'failed': "move_to_source", 'success':'move_to_source'})
        

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_PPTs')
    sis.start()
    rospy.loginfo("starting!")

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

    
