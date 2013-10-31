#!/usr/bin/env python

import roslib
roslib.load_manifest('slaw_manipulation')

import rospy
import tf

import math
import actionlib

import numpy as np
from sklearn.gaussian_process import GaussianProcess

from matplotlib import pyplot as pl

from slaw_arm_navigation.msg import *

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

base_frame = '/arm_link_0'
end_effector_frame = '/arm_link_5'

joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]


arm_up_s = [0.896244, 1.04883, -2.43523, 1.73184, 0.2]
arm_up_f = [0.011, 1.04883, -2.43523, 1.73184, 0.2]

arm_up_traj = [arm_up_s, arm_up_f]

start_pos = [0.891920306610033, 0.8952028681647454, -0.7625744927691185, 2.2020794590310313, 2.9601324920070926]



class GPforGripping:
    def __init__(self):
        self.tfListen = tf.TransformListener()
        rospy.sleep(1)
        self.targetArray = rospy.Subscriber('/vision/targets', TargetArray, self.cbTargetArray)
        self.pose_pub = rospy.Publisher('/vision/found', PoseStamped)

        self.gotPose = False
        self.trajectories = []
        self.pose_list = []
        self.start = False
        
        self.youbotIKAction_client = actionlib.SimpleActionClient('ik_action_arm', YoubotIKAction)
        rospy.logdebug('Waiting for action server to start')
        self.youbotIKAction_client.wait_for_server(rospy.Duration(10.0))
        self.gripper_client = actionlib.SimpleActionClient('gripper_action', TuckArmAction)
        rospy.logdebug('Waiting for action server to start')
        self.gripper_client.wait_for_server(rospy.Duration(10.0))
        rospy.logdebug('Sending goal to action server')

        self.tuck_arm_client = actionlib.SimpleActionClient('tuck_arm', TuckArmAction)
        rospy.logdebug('Waiting for action server to start')
        self.tuck_arm_client.wait_for_server(rospy.Duration(10.0))
        rospy.logdebug('Sending goal to action server')
        rospy.loginfo('all action clients connected')


    def gripperClose(self, close_grip):
        grip = TuckArmGoal()
        grip.tuck_gripper = close_grip
        self.gripper_client.send_goal_and_wait(grip, rospy.Duration(30.0), rospy.Duration(5.0))

    def tuckArm(self, tuck_arm):
        tuck = TuckArmGoal()
        tuck.tuck_arm = tuck_arm
        self.tuck_arm_client.send_goal_and_wait(tuck, rospy.Duration(30.0), rospy.Duration(5.0))

        
    def newTrajectory(self):
        print self.pose_list
        self.trajectories.append(self.pose_list)

        self.gotPose = False
        self.pose_list = []
        self.start = False
        self.cur_t = 0
        
    def cbTargetArray(self, msg):
        if self.gotPose or not self.start:
            return
        for t in msg.targets:
            if t.confidence>0.92:
                pose_in = PoseStamped()
                pose_in.pose = t.pose
                pose_in.header.frame_id = msg.header.frame_id
                if self.tfListen.frameExists(pose_in.header.frame_id) and self.tfListen.frameExists(base_frame):
                    time = self.tfListen.getLatestCommonTime(pose_in.header.frame_id, base_frame)
                    pose_in.header.stamp = time
                    self.pose = self.tfListen.transformPose(base_frame, pose_in)
                    self.pose_pub.publish(self.pose)
                    self.gotPose = True

    def cbTf(self):
        if not self.gotPose or not self.start:
            return
        
        if self.tfListen.frameExists(end_effector_frame) and self.tfListen.frameExists(base_frame):
            t = self.tfListen.getLatestCommonTime(base_frame, end_effector_frame)
            position, quaternion = self.tfListen.lookupTransform(base_frame, end_effector_frame, t)

            pose_dict = {}
            pose_dict['position'] = position
            pose_dict['orientation'] = quaternion
            pose_dict['time'] = t - self.t_0
            pose_dict['object_pose'] = self.pose
            self.pose_list.append(pose_dict)
            #print pose_dict
            #print position, quaternion

    def calcGPs(self):
        x = []
        y = []
        z = []
        r = []
        p = []
        j = []
       
        t = []
        for traj in self.trajectories:
            for p_dict in traj:
                
                #if len(t) > 0:
                #    if t[-1] == p_dict['time'].to_sec():
                #        print 'found double value'
                #        continue
                    
                x.append(p_dict['position'][0])# - p_dict['object_pose'].pose.position.x)
                y.append(p_dict['position'][1])# - p_dict['object_pose'].pose.position.y)
                z.append(p_dict['position'][2])# - p_dict['object_pose'].pose.position.z)
                roll,pitch,jaw = tf.transformations.euler_from_quaternion(p_dict['orientation'])

                r.append(roll)
                p.append(pitch)
                j.append(jaw)
                
                t.append(p_dict['time'].to_sec())


       
        self.x = np.array([x]).T.ravel()
        self.y = np.array([y]).T.ravel()
        self.z = np.array([z]).T.ravel()
        self.r = np.array([r]).T.ravel()
        self.p = np.array([p]).T.ravel()
        self.j = np.array([j]).T.ravel()

        self.t = np.array([t]).T

        
        #dx = 0.5 + 1.0 * np.random.random(x.shape)
        # self.gp_x = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                      thetaL=1e-3, thetaU=1,
        #                      #nugget=(dx / self.x) ** 2,
        #                      random_start=100)
        # self.gp_x.fit(self.t,self.x)

        # self.gp_y = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                             thetaL=1e-3, thetaU=1,
        #                             #nugget=(dy / self.y) ** 2,
        #                             random_start=100)
        # self.gp_y.fit(self.t,self.y)

        
        # self.gp_z = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                      thetaL=1e-3, thetaU=1,
        #                      #nugget=(dz / self.z) ** 2,
        #                      random_start=100)
        # self.gp_z.fit(self.t,self.z)

        
        # self.gp_r = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                      thetaL=1e-3, thetaU=1,
        #                      #nugget=(dr / self.r) ** 2,
        #                      random_start=100)
        # self.gp_r.fit(self.t,self.r)

        
        # self.gp_p = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                      thetaL=1e-3, thetaU=1,
        #                      #nugget=(dr / self.r) ** 2,
        #                      random_start=100)
        # self.gp_p.fit(self.t,self.p)

        # self.gp_j = GaussianProcess(corr='squared_exponential', theta0=1e-1,
        #                      thetaL=1e-3, thetaU=1,
        #                      #nugget=(dr / self.r) ** 2,
        #                      random_start=100)
        # self.gp_j.fit(self.t,self.j)

        # self.x_pred, MSE = self.gp_x.predict(self.t, eval_MSE = True)
        # self.y_pred, MSE = self.gp_y.predict(self.t, eval_MSE = True)
        # self.z_pred, MSE = self.gp_z.predict(self.t, eval_MSE = True)

        # self.r_pred, MSE = self.gp_r.predict(self.t, eval_MSE = True)
        # self.p_pred, MSE = self.gp_p.predict(self.t, eval_MSE = True)
        # self.j_pred, MSE = self.gp_j.predict(self.t, eval_MSE = True)

        # self.x_pred = self.x_pred.ravel()
        # self.y_pred = self.y_pred.ravel()
        # self.z_pred = self.z_pred.ravel()

        # self.r_pred = self.r_pred.ravel()
        # self.p_pred = self.p_pred.ravel()
        # self.j_pred = self.j_pred.ravel()



        print "calculation done"

    def switchOffMotors(self):
         rospy.wait_for_service('/arm_1/switchOffMotors')
         switch_off = rospy.ServiceProxy('/arm_1/switchOffMotors', Empty)
         try:
             switch_off()
             print 'switch off motors'
         except rospy.ServiceException, e:
             print "Service call failed: %s"%e

    def plot(self):
        pl.figure()
        pl.plot(self.t.ravel(),self.x.ravel(), ':r')
        pl.plot(self.t.ravel(),self.x_pred, 'b')

        pl.figure()
        pl.plot(self.t.ravel(),self.y.ravel(), ':r')
        pl.plot(self.t.ravel(),self.y_pred, 'b')

        pl.figure()
        pl.plot(self.t.ravel(),self.z.ravel(), ':r')
        pl.plot(self.t.ravel(),self.z_pred, 'b')

        
        pl.figure()
        pl.plot(self.t.ravel(),self.r.ravel(), ':r')
        pl.plot(self.t.ravel(),self.r_pred, 'b')

        pl.figure()
        pl.plot(self.t.ravel(),self.p.ravel(), ':r')
        pl.plot(self.t.ravel(),self.p_pred, 'b')

        pl.figure()
        pl.plot(self.t.ravel(),self.j.ravel(), ':r')
        pl.plot(self.t.ravel(),self.j_pred, 'b')

        pl.show()



    def run(self):
        self.cur_t = 0
        for t in self.t:
            self.moveToNextPos()
            rospy.sleep(0.5)
        
    def moveToNextPos(self):
        
        print "move to next pos"
        
        #pose = [self.x[self.cur_t] + self.pose.pose.position.x, self.y[self.cur_t] + self.pose.pose.position.y, self.z[self.cur_t] + self.pose.pose.position.z]
        pose = [self.x[self.cur_t], self.y[self.cur_t], self.z[self.cur_t]]
        quat = tf.transformations.quaternion_from_euler(self.r[self.cur_t],self.p[self.cur_t],self.j[self.cur_t])

        print pose, quat


        self.cur_t += 1

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.header.stamp = rospy.Time.now()


        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
                
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]


        
        self.pose_pub.publish(pose_stamped)
        
        self.sendGoal(pose,quat)


    def go(self, positions, wait = True):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [x for x in joint_names]
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                          velocities = [],
                                                          accelerations = [],
                                                          time_from_start = rospy.Duration((count+1) * self.move_duration)))
            goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
  
            if wait:
                self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
            else:
                self.arm_joint_client.send_goal(goal)

        
    def sendGoal(self, pose, quat):
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
             
        self.youbotIKAction_client.send_goal(goal)
        self.youbotIKAction_client.wait_for_result(rospy.Duration(5.0))
        print self.youbotIKAction_client.get_result()

    def processKey(self, key):
        if key == 'q\n':
            print "quit"
        if key == 'start\n':
            print "starting"
            self.start = True
            self.t_0 = rospy.Time.now()
        if key == 'stop\n':
            print "saving and stopping trajectory"
            self.newTrajectory()

        if key == 'calc\n':
            self.calcGPs()

        if key == '+\n':
            self.cur_t += 10
            if self.cur_t >= len(self.t):
                self.cur_t = len(self.t) - 1 
            
            print self.cur_t
        if key == '-\n':
            self.cur_t -= 10
            if self.cur_t < 0:
                self.cur_t = 0
            print self.cur_t
           
        if key == 'next\n':
            self.moveToNextPos()

        if key == 'run\n':
            self.run()
            
        if key == 'plot\n':
            self.plot()
            
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



def main():
    rospy.init_node("Gaussian_Gripping")
    rospy.sleep(0.001)  # wait for time
    gp_node = GPforGripping()

    rate = rospy.Rate(5)


    while not rospy.is_shutdown():
        gp_node.cbTf()
        i,o,e = select.select([sys.stdin],[],[],0.0001)
        for s in i:
            if s == sys.stdin:
                input_cmd = sys.stdin.readline()
                gp_node.processKey(input_cmd)
        rate.sleep()   

if __name__ == '__main__':
    main()

