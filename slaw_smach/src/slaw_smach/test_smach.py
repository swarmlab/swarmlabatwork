#!/usr/bin/env python

from ArmStates import *
from MoveStates import *
from ObjectDetectState import *

def test_scan():
    rospy.init_node('bmt_smach_test')
    sm = smach.StateMachine(outcomes=['end'])


    sm.userdata.pose = "S5"
    sm.userdata.object = "M20"
    sm.userdata.suffix = "_grip"

  
    with sm:
        #        smach.StateMachine.add('Recover', RecoverState(), transitions = {'done':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})
       
        smach.StateMachine.add('ScanMatcherPrePlace', ScanMatcher(), transitions = {'reached':'PreScanHole', 'not_reached':'ScanMatcherPrePlace', 'failed':'PreScanHole'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})


        smach.StateMachine.add('PreScanHole', PreGrip(), transitions = {'success':'ScanHole', 'failed':'TuckArmPreScan'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('TuckArmPreScan', TuckArm(), transitions = {'success':'PreScanHole', 'not_reached':'TuckArmPreScan','failed':'end'})

        #smach.StateMachine.add('MoveBack20', MoveBack(), transitions = {'done':'Place'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})
        smach.StateMachine.add("ScanHole", ScanForHoles(), transitions = {'success': 'FinePlace', 'failed':'ScanMatcherPrePlace','nothing_found': 'ScanMatcherPrePlace'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'object_in':'object', 'point_out':'point'})

                               

        smach.StateMachine.add('FinePlace', FinePlace(), transitions = {'success':'end', 'failed':'TuckArmFailPlace', 'too_far':'ScanMatcherPrePlace','failed_after_place':'TuckArmFailPlace'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'point_in':'point'})

        smach.StateMachine.add('TuckArmFailPlace', TuckArm(), transitions = {'success':'FinePlace', 'not_reached':'TuckArmFailPlace','failed':'end'})




        #smach.StateMachine.add('MoveToStart', MoveStateUserData(), transitions = {'reached': 'ScanMatcher', 'not_reached': 'RecoverToStart', 'failed': 'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        #smach.StateMachine.add('RecoverToStart', RecoverState(), transitions = {'done':'MoveToStart'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        
        #smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'PreGrip', 'not_reached':'ScanMatcher', 'failed':'MoveToStart'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

        #smach.StateMachine.add('PreGrip', PreGrip(), transitions = {'success':'Scan', 'failed':'TuckArmPreGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

        #smach.StateMachine.add('TuckArmPreGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArmPreGrip','failed':'end'})

        #smach.StateMachine.add("Scan", ScanForObjectsState(), transitions = {'success': 'Grip', 'failed': 'MoveToEnd'}, remapping = {'pose_in':'pose', 'pose_out':'pose', 'object_out':'object', 'point_out':'point'})
        
        #smach.StateMachine.add("Grip", Grip(), transitions = {'success':'MoveAwayFromPlatform', 'too_far':'ScanMatcher', 'failed':'TuckArmFailGrip', 'failed_after_grip':'TuckArmGrip'}, remapping = {'pose_in':'pose', 'object_in':'object', 'point_in':'point','pose_out':'pose', 'object_out':'object', 'point_out':'point'})
        
        #smach.StateMachine.add('TuckArmGrip', TuckArm(), transitions = {'success':'MoveAwayFromPlatform', 'not_reached':'TuckArmGrip','failed':'end'})

        #smach.StateMachine.add('TuckArmFailGrip', TuckArm(), transitions = {'success':'MoveToStart', 'not_reached':'TuckArmFailGrip','failed':'end'})

        #smach.StateMachine.add('MoveAwayFromPlatform', RecoverState(), transitions = {'done':'MovePrePlace'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        #smach.StateMachine.add('MovePrePlace', MoveStateUserData(), transitions = {'reached': 'ScanMatcherPrePlace', 'not_reached': 'MoveAwayFromPlatform', 'failed': 'MoveAwayFromPlatform'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        #smach.StateMachine.add('ScanMatcherPrePlace', ScanMatcher(), transitions = {'reached':'MoveBack20', 'not_reached':'ScanMatcherPrePlace', 'failed':'MoveBack20'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})

        # smach.StateMachine.add('MoveBack20', MoveBack(), transitions = {'done':'Place'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        # smach.StateMachine.add('Place', Place(), transitions = {'success':'MoveToStart', 'failed':'TuckArmFailPlace'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})

        # smach.StateMachine.add('TuckArmFailPlace', TuckArm(), transitions = {'success':'MoveToStart', 'not_reached':'TuckArmFailPlace','failed':'end'})
        # smach.StateMachine.add('RecoverToEnd', RecoverState(), transitions = {'done':'MoveToEnd'}, remapping = {'pose_in':'pose', 'pose_out': 'pose'})

        # smach.StateMachine.add('MoveToEnd', MoveStateUserData(), transitions = {'reached': 'end', 'not_reached': 'RecoverToEnd', 'failed': 'end'}, remapping = {'pose_in':'pose', 'pose_out':'pose'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_TEST')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    #print sm.userdata.object
    #print sm.userdata.point
    rospy.spin()
    sis.stop()
    

def main():
    rospy.init_node('arm_smach_test')

    sm = smach.StateMachine(outcomes=['reached', 'failed'])

    sm.userdata.pose = "test"
    sm.userdata.suffix = "_grip"

    #sm.userdata.point = ""
    #sm.userdata.object = "m20"

    
    with sm:
        #smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'success':'PreKinect', 'failed':'ScanMatcher'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})
        #smach.StateMachine.add('ScanMatcher', ScanMatcher(), transitions = {'reached':'Detect', 'not_reached':'MoveStateSmart', 'failed':'MoveStateSmart'}, remapping = {'pose_in':'pose', 'suffix_in':'suffix', 'pose_out':'pose'})


        #smach.StateMachine.add('KinectDetect', KinectDetectionState(), transitions = {'success':'reached', 'failed':'failed'}, remapping = {'pose_in':'pose','pose_out':'pose','object_out': 'object', 'point_out':'point'})

        smach.StateMachine.add('TuckArm', TuckArm(), transitions = {'success':'PreKinect', 'not_reached':'TuckArm','failed':'failed'})
        
        smach.StateMachine.add('PreKinect', PreKinect(), transitions = {'success':'TuckArmTest', 'failed':'TuckArmKin'},remapping = {'pose_in':'pose', 'pose_out':'pose'})

        smach.StateMachine.add('TuckArmKin', TuckArm(), transitions = {'success':'PreKinect', 'not_reached':'TuckArmKin','failed':'failed'})

        smach.StateMachine.add('TuckArmTest', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArm','failed':'failed'})

        
        smach.StateMachine.add('PreGrip', PreGrip(), transitions = {'success':'reached', 'failed':'TuckArmGrip'},remapping = {'pose_in':'pose', 'pose_out':'pose'})
        smach.StateMachine.add('TuckArmGrip', TuckArm(), transitions = {'success':'PreGrip', 'not_reached':'TuckArm','failed':'failed'})
        
        #smach.StateMachine.add('grip', Grip, transitions = {'success':'reached', 'failed':'failed', 'too_far': 'failed'},remapping = {'pose_in':'pose', 'point_in': 'point', 'object_in':'object', 'offset_out':'offset','object_out':'object'  'pose_out':'pose'})
       
        #smach.StateMachine.add('place', Place, transitions = {'success':'reached', 'failed':'failed'},remapping = {'pose_in':'pose', 'pose_out':'pose'})
        


        
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
    #bnt()
    #main()
    test_scan()
