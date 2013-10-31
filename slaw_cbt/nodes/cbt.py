#!/usr/bin/env python
import roslib
roslib.load_manifest("slaw_smach")
import rospy

from slaw_smach.ArmStates import *
from slaw_smach.MoveStates import *
from slaw_smach.ObjectDetectState import *

def cbt():
    rospy.init_node('cbt_smach_test')
    sm = smach.StateMachine(outcomes=['end'])


    #sm.userdata.pose = "D2"
    locations = rospy.get_param('locations')
    sm.userdata.pose = locations[0]['name']

    sm.userdata.pose = 'C1'
    sm.userdata.suffix = "_grip"

    #pre-grsm.userdata.pose = "test"
    with sm:
        #        smach.StateMachine.add('Recover', RecoverState(), transitions = {'done':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        #smach.StateMachine.add('MoveBack', MoveBack(), transitions = {'done':'end'})

        smach.StateMachine.add('MoveToStart', MoveStateUserData(), transitions = {'reached': 'ScanMatcher', 'not_reached': 'RecoverToStart', 'failed': 'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('RecoverToStart', RecoverState(), transitions = {'done':'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        
        smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'PreGrip', 'not_reached':'ScanMatcher', 'failed':'PreGrip'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

        smach.StateMachine.add('PreGrip', PreGripCBT(), transitions = {'success':'ScanForObject', 'failed':'TuckArmPreGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('TuckArmPreGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArmPreGrip','failed':'end'})

        ## here comes the scanning state
        smach.StateMachine.add('ScanForObject', ScanForObjectCBT(), transitions = {'success':'GripCBT'})
        
        smach.StateMachine.add('GripCBT', GripCBT(), transitions = {'end':'end'})


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
    cbt()
    #main()

