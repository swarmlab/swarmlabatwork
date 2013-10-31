#!/usr/bin/env python
import roslib
roslib.load_manifest("slaw_smach")
import rospy

from slaw_smach.ArmStates import *
from slaw_smach.MoveStates import *
from slaw_smach.ObjectDetectState import *

def btt():
    rospy.init_node('btt_smach_test')
    sm = smach.StateMachine(outcomes=['end'])


    #sm.userdata.pose = "D2"
    locations = rospy.get_param('locations')
    sm.userdata.pose = locations[0]['name']
    sm.userdata.suffix = "_grip"

    #pre-grsm.userdata.pose = "test"
    with sm:
        #        smach.StateMachine.add('Recover', RecoverState(), transitions = {'done':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        #smach.StateMachine.add('MoveBack', MoveBack(), transitions = {'done':'end'})

        smach.StateMachine.add('MoveToStart', MoveStateUserData(), transitions = {'reached': 'ScanMatcher', 'not_reached': 'RecoverToStart', 'failed': 'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('RecoverToStart', RecoverState(), transitions = {'done':'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        
        smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'PreGrip', 'not_reached':'ScanMatcher', 'failed':'MoveToStart'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

        smach.StateMachine.add('PreGrip', PreGrip(), transitions = {'success':'Scan', 'failed':'TuckArmPreGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('TuckArmPreGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArmPreGrip','failed':'end'})

        smach.StateMachine.add("Scan", ScanForObjectsState(), transitions = {'success': 'Grip', 'failed':'TuckArmMoveStart','nothing_found': 'TuckArmDelete'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'object_out':'object', 'point_out':'point'})

        smach.StateMachine.add('TuckArmMoveStart', TuckArm(), transitions = {'success':'MoveToStart', 'not_reached':'TuckArmMoveStart','failed':'end'})

        
        smach.StateMachine.add('TuckArmDelete', TuckArm(), transitions = {'success':'DeleteState', 'not_reached':'TuckArmDelete','failed':'end'})

        #DeleteNode
        smach.StateMachine.add("DeleteState", DelReachedGoalStateBTT(), transitions = {'not_end':'MoveToStart', 'move_out':'MoveToEnd'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})
        
        smach.StateMachine.add("Grip", Grip(), transitions = {'success':'MoveAwayFromPlatform', 'too_far':'ScanMatcher', 'failed':'TuckArmFailGrip', 'failed_after_grip':'TuckArmGrip'}, remapping = {'pose_in':'pose', 'object_in':'object', 'point_in':'point','pose_out':'pose', 'object_out':'object', 'point_out':'point'})
        
        smach.StateMachine.add('TuckArmGrip', TuckArm(), transitions = {'success':'MoveAwayFromPlatform', 'not_reached':'TuckArmGrip','failed':'end'})

        smach.StateMachine.add('TuckArmFailGrip', TuckArm(), transitions = {'success':'MoveToStart', 'not_reached':'TuckArmFailGrip','failed':'end'})

        smach.StateMachine.add('MoveAwayFromPlatform', RecoverState(), transitions = {'done':'MovePrePlace'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        smach.StateMachine.add('MovePrePlace', MoveStateUserData(), transitions = {'reached': 'ScanMatcherPrePlace', 'not_reached': 'MoveAwayFromPlatform', 'failed': 'MoveAwayFromPlatform'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('ScanMatcherPrePlace', ScanMatcher(), transitions = {'reached':'Place', 'not_reached':'ScanMatcherPrePlace', 'failed':'Place'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

        #smach.StateMachine.add('MoveBack20', MoveBack(), transitions = {'done':'Place'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('Place', Place(), transitions = {'success':'RecoverToStart', 'failed':'TuckArmFailPlace'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('TuckArmFailPlace', TuckArm(), transitions = {'success':'Place', 'not_reached':'TuckArmFailPlace','failed':'end'})

        smach.StateMachine.add('RecoverToEnd', RecoverState(), transitions = {'done':'MoveToEnd'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        smach.StateMachine.add('MoveToEnd', MoveStateUserData(), transitions = {'reached': 'end', 'not_reached': 'RecoverToEnd', 'failed': 'end'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_TEST')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    #print sm.userdata.object
    #print sm.userdata.point
    rospy.spin()
    sis.stop()
    

if __name__ == '__main__':
    btt()
    #main()

