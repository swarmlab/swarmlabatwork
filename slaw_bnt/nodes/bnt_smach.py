#!/usr/bin/python

import roslib
roslib.load_manifest("slaw_smach")
import rospy

from slaw_smach.MoveStates import * 

if __name__ == "__main__":
    rospy.init_node('bnt_smach')
    sm = smach.StateMachine(outcomes=['end'])

    sm.userdata.pose = ""
    with sm:
#        smach.StateMachine.add('TuckArm', TuckArm(), transitions = {'success':'MoveStateSmart', 'not_reached':'MoveStateSmart','failed':'MoveStateSmart'})

        smach.StateMachine.add('MoveStateSmart', MoveStateSmart(get_next_goal_bnt, get_dir_bnt, get_suffix_bnt), transitions = {'reached': 'SleepState', 'not_reached':'Recover', 'failed':'DeleteNode', 'end':'end'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'suffix_out':'suffix'})

        smach.StateMachine.add('Recover', RecoverState(), transitions = {'done':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

#        smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'SleepState', 'not_reached':'MoveStateSmart', 'failed':'DeleteNode'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})
        smach.StateMachine.add('SleepState', SleepState(), transitions = {'done':'DeleteNode'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
        
        smach.StateMachine.add('DeleteNode', DelReachedGoalState(), transitions = {'done':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_TEST')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    #print sm.userdata.object
    #print sm.userdata.point
    rospy.spin()
    sis.stop()
