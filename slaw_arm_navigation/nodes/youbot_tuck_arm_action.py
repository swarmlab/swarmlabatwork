#!/usr/bin/env python

"""
usage: tuck_arm.py [-q] [-a ACTION]

Options:
  -q or --quit   Quit when finished
  -a or --action Action to do

Actions:
  t or tuck
  u or untuck

"""

import roslib
roslib.load_manifest('slaw_arm_navigation')

import rospy

import signal
import os
import sys
import time
import math
import getopt

import actionlib
from slaw_arm_navigation.msg import *

from trajectory_msgs.msg import *
from control_msgs.msg import *
from actionlib_msgs.msg import *
from sensor_msgs.msg import *

from std_srvs.srv import *

def usage():
  print __doc__ % vars()
  rospy.signal_shutdown("Help requested")


  
joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]

#arm_up_traj = [2.56244, 1.04883, -2.43523, 1.73184, 0.2]

arm_up = [0.011, 1.04883, -2.43523, 1.73184, 0.2]
arm_up_traj = [[0.011, 1.04883, -2.43523, 1.73184, 0.2]]
arm_tucked = [0.011, 0.011, -0.016, 0.023, 0.111]
arm_tucked_traj = [arm_tucked]

TUCKED = 0
NOT_TUCKED = 1

class TuckArmActionServer:
  def __init__(self, node_name):
    self.arm_received = False
    self.configuration = [0, 0, 0, 0, 0]


    self.node_name = node_name

    # arm state: -1 unknown, 0 tucked, 1 untucked
    self.arm_state = -1
    self.success = True

    # Get controller name and start joint trajectory action clients
    self.move_duration = rospy.get_param('~move_duration', 2.5)
    arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
    gripper_action_name = rospy.get_param('~gripper_joint_trajectory_action', '/arm_1/gripper_controller/joint_trajectory_action')
    
    self.arm_joint_client = arm_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
    self.gripper_joint_client = gripper_client = actionlib.SimpleActionClient(gripper_action_name, FollowJointTrajectoryAction)

    
    # Connect to controller state
    rospy.Subscriber('/joint_states', JointState, self.stateCb)

    # Wait for joint clients to connect with timeout
    if not self.arm_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("tuck_arms: arm_joint_client action server did not come up within timelimit")
    if not self.gripper_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("tuck_arms: gripper_joint_client action server did not come up within timelimit")


    self.server = rospy.Service("/tuck_arm", Empty, self.tuck)
    # Construct action server
    self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmAction, self.executeCB, False)
    self.action_server.start()


  def executeCB(self, goal):
    # Make sure we received arm state
    while not self.arm_received:
      rospy.sleep(0.1)
      if rospy.is_shutdown():
        return

    # Create a new result
    result = TuckArmResult()
    result.tuck_arm = goal.tuck_arm

    # Tucking left and right arm
    if goal.tuck_arm:
      rospy.loginfo('Tucking arm...')
      self.tuck()

    # UnTucking both arms
    if not goal.tuck_arm:
      rospy.loginfo("Untucking arms...")
      self.untuck()


    # Succeed or fail
    if self.success:
      result.tuck_arm = goal.tuck_arm
      #result.tuck_gripper = goal.tuck_gripper
      self.action_server.set_succeeded(result)
    else:
      rospy.logerr("Tuck or untuck arms FAILED: Arm value: %d. Gripper value: %d" % (result.tuck_arm, result.tuck_gripper))
      result.tuck_arm = (self.arm_state == 0)
      #result.tuck_gripper = False
      self.action_server.set_aborted(result)


  # clears r arm and l arm
  def tuck(self, req = None):
    if self.arm_state != TUCKED:
        arm_up_with_out_turn = [x for x in arm_up_traj[0]]
        arm_up_with_out_turn[0] = self.configuration[0]


        confs = []
        confs.append(arm_up_with_out_turn)
        confs.append(arm_up)
        confs.append(arm_tucked)

        #print confs
        self.go(confs)

        #self.go([arm_up_with_out_turn, arm_up_traj, arm_tucked_traj])
        #self.go(arm_up_traj)
        #self.go(arm_tucked_traj)
    if req is not None:
      return EmptyResponse()
    
  def untuck(self):
    if self.arm_state != NOT_TUCKED:
        arm_up_with_out_turn = [[x for x in arm_up_traj[0]]]
        arm_up_with_out_turn[0][0] = self.configuration[0]

        confs = arm_up_with_out_turn

        confs.append(arm_up)
        
        self.go(confs)
#        self.go(arm_up_traj)

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
    #print goal
    if wait:
      if not self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0)):
        self.success = False
    else:
      self.arm_joint_client.send_goal(goal)


  # Returns angle between -pi and + pi
  def angleWrap(self, angle):
    while angle > math.pi:
      angle -= math.pi*2.0
    while angle < -math.pi:
      angle += math.pi*2.0
    return angle


  def stateCb(self, msg):
    arm_msg = False
    sum_tucked = 0
    for k in range(5):
      for i in range(len(msg.name)):
        joint_name = "arm_joint_" + str(k + 1)
        if(msg.name[i] == joint_name):
          #print msg.name[i]
          self.configuration[k] = msg.position[i]
          sum_tucked += math.fabs(self.configuration[k]-arm_tucked[i])
          arm_msg = True
#          print math.fabs(self.configuration[k])
#          print math.fabs(arm_tucked[i])
    if arm_msg:
      if sum_tucked >= 0 and sum_tucked < 0.2:
        self.arm_state = TUCKED
      else:
        self.arm_state = NOT_TUCKED
      self.arm_received = True
#      print sum_tucked
#      print self.arm_state

def main():
    action_name = 'tuck_arm'
    rospy.init_node(action_name)
    rospy.sleep(0.001)  # wait for time
    tuck_arm_action_server = TuckArmActionServer(action_name)

    quit_when_finished = False

      # check for command line arguments, and send goal to action server if required
    myargs = rospy.myargv()[1:]
    if len(myargs):
        goal = TuckArmGoal()
        goal.tuck_arm = True
        opts, args = getopt.getopt(myargs, 'hqa:', ['quit','action'])
        for opt, action in opts:
          
          if opt in ('-a', '--action'):
              if action in ('tuck', 't'):
                  goal.tuck_arm = True
              elif action in ('untuck', 'u'):
                  goal.tuck_arm = False
              else:
                  rospy.logerr('Invalid action for right arm: %s'%action)
                  rospy.signal_shutdown("ERROR")
          if opt in ('--quit', '-q'):
              quit_when_finished = True

          if opt in ('--help', '-h'):
              usage()
          
        tuck_arm_client = actionlib.SimpleActionClient(action_name, TuckArmAction)
        rospy.logdebug('Waiting for action server to start')
        tuck_arm_client.wait_for_server(rospy.Duration(10.0))
        rospy.logdebug('Sending goal to action server')
        tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

        if quit_when_finished:
          rospy.signal_shutdown("Quitting")
    rospy.spin()

if __name__ == '__main__':
    main()

