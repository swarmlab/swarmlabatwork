#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_manipulation')

import rospy
import tf

import math
import actionlib

import numpy as np

from slaw_arm_navigation.msg import *
from slaw_arm_navigation.srv import *

from trajectory_msgs.msg import *
from control_msgs.msg import *
from actionlib_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

from slaw_vision.msg import *

import kinematics_msgs.srv
import kinematics_msgs.msg
import arm_navigation_msgs.msg
import arm_navigation_msgs.srv

import sys
import select

from std_srvs.srv import Empty

from slaw_manipulation.msg import PoseStampedLabeled

pre_kinect = [5.323699090192345, 1.3339887113233428, -2.446201119717692, 1.8014157762386995, 2.9404422281923392]

pre_grip = [5.323699090192345, 1.134883, -4.119026326794897, 3.3593047267948966, 2.923]


base_frame = '/arm_base_link'
end_effector_frame = '/arm_tip_link'
joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

arm_up_pose = [0.011, 1.04883, -2.43523, 1.73184, 0.2]

start_pos = [0.891920306610033, 0.8952028681647454, -0.7625744927691185, 2.2020794590310313, 2.9601324920070926]

pre_left_up = [(0.017551446379286594, 0.2263433823699852, 0.4012978132268241), (-0.19679105780188727, 0.19711996879047927, 0.6987387858995414, 0.6589242039513267)]
pre_left_down = [(0.028064046300264385, 0.24060391704734185, 0.10983889387519954), (-0.6907682595353951, 0.7228825847466658, 0.011900449777350445, 0.011762635058405368)]


arm_tucked = [0.011, 0.011, -0.016, 0.023, 0.111]

WAIT_FOR_SERVERS = 2.0

GRIPPER_LENGTH = 0.087

TABLE_HEIGHT = 0.105
ABOVE_TABLE = 0.002 #MM above table height
PRE_GRIP_HEIGHT = 0.04

STEP_SIZE = 0.05
GRIP_X = 0.21




class Gripping:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        #listen for some time
        rospy.sleep(2)

        self.targetArray = rospy.Subscriber('/vision/targets', TargetArray, self.cbTargetArray)
        self.pose_pub = rospy.Publisher('/vision/found', PoseStamped)
        self.polygon_pub = rospy.Publisher('/vision/rect', PolygonStamped)

        self.gotPose = False
        self.stopRecPose = False
        self.gotArmConf = False
        self.pose = None

        self.arm_base_link_height = rospy.get_param('arm_base_link_height')
        
        self.joints = rospy.get_param("joints")
        self.mount_offset = rospy.get_param("arm_rot_offset")

        self.configuration = [0,0,0,0,0]
        # Connect to controller state
        rospy.Subscriber('/joint_states', JointState, self.stateCb)


        #Gripper server
        self.gripper_client = actionlib.SimpleActionClient('gripper_action', TuckArmAction)
        if not self.gripper_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVERS)):
            rospy.logerr("gripper action server did not come up within timelimit")

        #Tuckarm Server
        self.tuck_arm_client = actionlib.SimpleActionClient('tuck_arm', TuckArmAction)
        if not self.tuck_arm_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVERS)):
            rospy.logerr("tuck arm action server did not come up within timelimit")

        #Arm joint action server
        self.move_duration = rospy.get_param('~move_duration', 2.5)
        arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
        #arm_action_name = "/arm_1/arm_controller/follow_joint_trajectory"
        self.arm_joint_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
        if not self.arm_joint_client.wait_for_server(rospy.Duration(WAIT_FOR_SERVERS)):
            rospy.logerr("arm_joint_client action server did not come up within timelimit")

        self.iks = rospy.ServiceProxy('/arm_1/simple_ik_server', SimpleIkSolver)

        rospy.Subscriber("/vision/detected_object", PoseStampedLabeled, self.cb_detect)
        rospy.loginfo('initialized')



    def cb_detect(self,msg):
        if not self.stopRecPose:
            self.detect_pose = msg.pose.pose
        

    def rotate_only(self, side, turned = False):
        rotate_only = [x for x in self.configuration]
        straight = self.joints['arm_joint_1']['straight'] - self.mount_offset

        
        rotate_only[0] = straight
        print straight
        if (side == 'left'):
            rotate_only[0] += -math.pi/2
        elif (side == 'right'):
            rotate_only[0] += +math.pi/2
        elif (side == 'back'):
            rotate_only[0] += math.pi
    
        if turned:
            rotate_only[0] = self.limit_joint1_ang(rotate_only[0]+math.pi)
        return rotate_only


    def limit_joint1_ang(self,ang):
        while ang < self.joints['arm_joint_1']['min']:
            ang = ang + 2*math.pi
        while ang > self.joints['arm_joint_1']['max']:
            ang = ang - 2* math.pi
        return ang

        
    #grip from visual detection
    def gripDetect(self):
        if self.detect_pose is None:
            return

        height = TABLE_HEIGHT + ABOVE_TABLE - self.arm_base_link_height
        point = [GRIP_X, 0.0, height] #0.28 #0.15

        quat = [self.detect_pose.orientation.x, self.detect_pose.orientation.y, self.detect_pose.orientation.z,self.detect_pose.orientation.w]

        r, p, y = tf.transformations.euler_from_quaternion(quat)

        point[0] += self.detect_pose.position.x
        point[1] -= self.detect_pose.position.y #+ 0.02375

        gripX = point[0]
        conf = np.array(self.call_ik_solver(point, side = 'left'))

        if conf is None:
            print 'too_far'
            return 
        conf[4] = self.getAngle(conf[4], y)

        point[2]+= PRE_GRIP_HEIGHT


        #while conf2 is None:
        #    conf2 = np.array(self.call_ik_solver(point, side = 'left'))
        #    point[0]-= 0.01 #get closer to solve pre-grip
        #conf2[4] = conf[4]

        confs = []
        rotate = self.rotate_only(side = 'left')
                
        confs.append(rotate)
        first = None
        while point[2] > height:
            point[2] -= 0.004
            point[0] = gripX
            conf2 = None

            while conf2 is None:
                conf2 = np.array(self.call_ik_solver(point, side = 'left'))
                point[0]-= 0.01 #get closer to solve pre-grip
            conf2[4] = conf[4]
            if first is None:
                first = [x for x in conf2]
            confs.append(conf2)

        #conf2[3] += 0.1# adjust for overshoot!

        self.gripperClose(False)
        confs.append(conf)
        print rotate, conf, conf2
        self.go(confs)


        tucked = self.rotate_only(side = 'left')
        tucked[1] = self.joints['arm_joint_2']['min']
        tucked[2] = self.joints['arm_joint_3']['min']
        tucked[3] = self.joints['arm_joint_4']['min']

        
        self.gripperClose(True)
        self.go([first])


    def getAngle(self, straight, angle):
        ang = straight - angle
        #print straight, angle
        while ang < straight-math.pi/2:
            ang = ang + math.pi
        while ang > straight+math.pi/2:
            ang = ang - math.pi
        return ang

        
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

    def tuckArm(self, tuck_arm):
        tuck = TuckArmGoal()
        tuck.tuck_arm = tuck_arm
        self.tuck_arm_client.send_goal_and_wait(tuck, rospy.Duration(30.0), rospy.Duration(5.0))
        
    def cbTargetArray(self, msg):
        #print msg
        for t in msg.targets:
            if t.confidence>92:
                pose_in = PoseStamped()
                pose_in.pose = t.pose
                pose_in.header.frame_id = msg.header.frame_id
                if self.tfListen.frameExists(pose_in.header.frame_id) and self.tfListen.frameExists(base_frame):
                    time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, base_frame)
                    pose_in.header.stamp = time
                    self.gotPose = True
                   
                    polygon = PolygonStamped()
                    polygon.header = msg.header
                    #polygon.polygon = t.hull
                    polygon.polygon = t.rectangle
                    self.polygon_pub.publish(polygon)

                    vects = []
                    for p,idx in zip(polygon.polygon.points, range(len(polygon.polygon.points))):
                        idx_two = idx+1
                        if idx_two == len(polygon.polygon.points):
                            idx_two = 0
                        p2 = polygon.polygon.points[idx_two]
                        vects.append(np.array([p2.x - p.x, p2.y - p.y]))

                    max_len = 0
                    mex_vect = None
                    for vect in vects:
                        if np.linalg.norm(vect)>max_len:
                            max_vect = vect
                            max_len = np.linalg.norm(vect)
                    if max_vect is not None:
                        ang = math.atan2(max_vect[1], max_vect[0])

                        r, p, y = tf.transformations.euler_from_quaternion(pre_left_down[1])
                        quat = tf.transformations.quaternion_from_euler(r,p,ang)
                    
                        if not self.stopRecPose:
                            self.pose = self.tfListen.transformPose(base_frame, pose_in)
                            self.pose.pose.orientation.x = quat[0]
                            self.pose.pose.orientation.y = quat[1]
                            self.pose.pose.orientation.z = quat[2]
                            self.pose.pose.orientation.w = quat[3]
                    self.pose_pub.publish(self.pose)
                else:
                    print "tf_lookup failed"
                    
    def switchOffMotors(self):
         rospy.wait_for_service('/arm_1/switchOffMotors')
         switch_off = rospy.ServiceProxy('/arm_1/switchOffMotors', Empty)
         try:
             switch_off()
             print 'switch off motors'
         except rospy.ServiceException, e:
             print "Service call failed: %s"%e

       
    def recordTF(self):
        print "try tf"
        if self.tfListen.frameExists(end_effector_frame) and self.tfListen.frameExists(base_frame):
            t = self.tfListen.getLatestCommonTime(base_frame, end_effector_frame)
            position, quaternion = self.tfListen.lookupTransform(base_frame, end_effector_frame, t)
            print "Position:"
            print position
            self.position = position
            print "Quat:"
            print quaternion
            self.quaternion = quaternion
            print 'rpy'
            r, p, y = tf.transformations.euler_from_quaternion(quaternion)
            print r,p,y
      
        else:
            print 'tf error'

    def armUp(self):
        if not self.gotArmConf:
            print 'did not get arm conf yet'
            return
        arm_up_without_turn = [[x for x in arm_up_pose]]
        arm_up_without_turn[0][0] = self.configuration[0]
        arm_up_without_turn[0][4] = self.configuration[4]
             
        self.go(arm_up_without_turn)


    def go(self, positions):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [x for x in joint_names]
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                          velocities = [],
                                                          accelerations = [],
                                                          time_from_start = rospy.Duration((count+1) * self.move_duration)))
            
            goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
        self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

        res = self.arm_joint_client.get_result()
        print res
        print self.arm_joint_client.get_state()

        return True
        

    def call_ik_solver(self, goal_point, side = 'front'):
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
            resp = self.iks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))

        return resp.joints


    #calibration grip

    def testGrip(self, offset):
        #self.gripperClose(False)
        #self.gripperClose(True)
        #rospy.sleep(2)

        height = TABLE_HEIGHT + ABOVE_TABLE - self.arm_base_link_height

        point = [GRIP_X, offset, height] #0.28 #0.15


        conf = np.array(self.call_ik_solver(point, side = 'left'))

        point =  [GRIP_X, offset,  0.0] #0.28 #0.15

        
        conf2 = np.array(self.call_ik_solver(point, side = 'left'))
    #    conf2[3] = conf[3]#
    
        self.go([conf2, conf])
        
        self.gripperClose(False)

        self.go([conf2])


    def gripTestFromKinect(self):

        self.gripperClose(False)
        #arm up
        arm_up_without_turn = [x for x in arm_up_pose]
        arm_up_without_turn[0] = self.configuration[0]
        arm_up_without_turn[4] = self.configuration[4]
               
       

        pose = [self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z+0.03]
        conf = np.array(self.call_ik_solver(pose, side = 'left'))

        #first rotate
        rotate = [x for x in arm_up_without_turn]
        rotate[0] = conf[0]
        
        #pre grip a bit higher
        point = [pose[0], pose[1], pose[2]+0.02]

        #adjust last pose
        conf2 = np.array(self.call_ik_solver(point, side = 'left'))
        conf2[3] = conf[3]

        

        #print conf, arm_up_without_turn, roate
        self.go([arm_up_without_turn, rotate, conf2, conf])

        self.gripperClose(True)

        #self.go([conf2])

        
    def playRecordedTF(self):
        print 'trying to go to'
        print self.position
        
        conf = self.call_ik_solver(self.position)
        self.go([conf])

    def pre_kinect(self):
        self.armUp()
        self.go([pre_kinect])

        
    def pre_grip(self):
        self.armUp()
        self.go([pre_grip])
        
        
    def armTuck(self):
        if not self.gotArmConf:
            print 'did not get arm conf yet'
            return
        arm_up_without_turn = [[x for x in arm_tucked]]
        arm_up_without_turn[0][0] = self.configuration[0]
        arm_up_without_turn[0][4] = self.configuration[4]
             
        return self.go(arm_up_without_turn)

            
    def processKey(self, key):
        if key == 'joints\n':
            print self.configuration
            self.joints = [x for x in self.configuration]
        if key == '+\n':
            turn = [x for x in self.configuration]
            turn[4] += 0.1
            self.go([turn])
        if key == '-\n':
            turn = [x for x in self.configuration]
            turn[4] -= 0.1
            self.go([turn])
        if key == 'grip\n':
            self.grip()
        if key == 'calibrate-left\n':
            self.testGrip(0.10)
        if key == 'calibrate-right\n':
            self.testGrip(-0.10)
        if key == 'calibrate-center\n':
            self.testGrip(-0.00)

        if key == 'test\n':
            self.gripTest()
        if key == 'place\n':
            self.place()
        if key == 'place-right\n':
            self.place(side = 'right')
        if key == 'tuck\n':
            self.tuckArm(True)
        if key == 'untuck\n':
            self.tuckArm(False)
        if key == 'open\n':
            self.gripperClose(False)
        if key == 'close\n':
            self.gripperClose(True)
        if key == 'off\n':
            self.switchOffMotors()

        if key == 'pose\n':
            print self.detect_pose
            print 'rpy'
            r, p, y = tf.transformations.euler_from_quaternion([self.detect_pose.orientation.x, self.detect_pose.orientation.y, self.detect_pose.orientation.z, self.detect_pose.orientation.w])
            print r,p,y
        
        if key == 'pre-grip\n':
            self.pre_grip()
        if key == 'pre-kinect\n':
            self.pre_kinect()

        if key == 'grip-detect\n':
            self.gripDetect()

        if key == 'tf\n':
            self.recordTF()
        if key == 'try-tf\n':
            self.playRecordedTF()
        if key == 'play-joints\n':
            self.go([self.joints])

        if key == 'stop\n':
            self.stopRecPose = True
        if key == 'start\n':
            self.stopRecPose = False
        if key == 'arm-up\n':
            self.armUp()
            
            
def main():
    rospy.init_node("Gripping")
    rospy.sleep(0.001)  # wait for time
    grip_node = Gripping()

    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
        i,o,e = select.select([sys.stdin],[],[],0.0001)
        for s in i:
            if s == sys.stdin:
                input_cmd = sys.stdin.readline()
                grip_node.processKey(input_cmd)
        rate.sleep()   

if __name__ == '__main__':
    main()

