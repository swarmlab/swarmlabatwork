#!/usr/bin/env python

"""
usage: gripper.py [-q] [-a ACTION]

Options:
  -q or --quit   Quit when finished
  -a or --action Action to do

Actions:
  o or open
  c or close
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


  
#joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]


joint_names = ["gripper_finger_joint_l","gripper_finger_joint_r"]
gripper_closed = [0, 0]
gripper_open = [0.0115, 0.0115]

gripper_closed_traj = [gripper_closed]
gripper_open_traj = [gripper_open]

CLOSED = 0
OPEN = 1
UNKNOWN = -1

class GripperAction:
  def __init__(self, node_name):
    self.arm_received = False
    self.configuration = [0, 0]


    self.node_name = node_name

    # arm state: -1 unknown, 0 closed, 1 open
    self.gripper_state = UNKNOWN
    self.success = True

    # Get controller name and start joint trajectory action clients
    self.move_duration = rospy.get_param('~move_duration', 2.5)
    gripper_action_name = rospy.get_param('~gripper_joint_trajectory_action', '/arm_1/gripper_controller/joint_trajectory_action')
    
    self.gripper_joint_client = gripper_client = actionlib.SimpleActionClient(gripper_action_name, FollowJointTrajectoryAction)

    
    # Connect to controller state
    rospy.Subscriber('/joint_states', JointState, self.stateCb)

    if not self.gripper_joint_client.wait_for_server(rospy.Duration(30)):
	    rospy.logerr("tuck_arms: gripper_joint_client action server did not come up within timelimit")

            
    self.serv_open = rospy.Service("/open_gripper", Empty, self.open)
    self.serv = rospy.Service("/close_gripper", Empty, self.close)
    
    # Construct action server
    self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmAction, self.executeCB, False)
    self.action_server.start()


  def executeCB(self, goal):
    # Make sure we received arm state
    while not self.gripper_received:
      rospy.sleep(0.1)
      if rospy.is_shutdown():
        return

    self.success = True
    # Create a new result
    result = TuckArmResult()
    result.tuck_gripper = goal.tuck_gripper

    if goal.tuck_gripper:
      rospy.loginfo('Close Gripper...')
      self.close()

    # UnTucking gripper
    if not goal.tuck_gripper:
      rospy.loginfo("open Gripper..")
      self.open()


    # Succeed or fail
    if self.success:
      result.tuck_gripper = goal.tuck_gripper
      self.action_server.set_succeeded(result)
    else:
      result.tuck_gripper = (self.gripper_state == CLOSED)
      self.action_server.set_aborted(result)


  def open(self, req = None):
    if self.gripper_state != OPEN:
      self.go(gripper_open_traj)
    if req is not None:
      return EmptyResponse()

    
  def close(self, req = None):
    if self.gripper_state != CLOSED:
      self.go(gripper_closed_traj)
    if req is not None:
      return EmptyResponse()

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
      if not self.gripper_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0)):
        self.success = False
    else:
      self.gripper_joint_client.send_goal(goal)


  # Returns angle between -pi and + pi
  def angleWrap(self, angle):
    while angle > math.pi:
      angle -= math.pi*2.0
    while angle < -math.pi:
      angle += math.pi*2.0
    return angle


  def stateCb(self, msg):
    gripper_msg = False
    sum_open = 0
    sum_closed = 0
    
    for i in range(len(msg.name)):
      if(msg.name[i] == joint_names[0]):
        self.configuration[0] = msg.position[i]
        sum_closed += math.fabs(self.configuration[0]-gripper_closed[0])
        sum_open +=math.fabs(self.configuration[0]-gripper_open[0])
        gripper_msg = True
      if(msg.name[i] == joint_names[1]):
        self.configuration[1] = msg.position[i]
        sum_closed += math.fabs(self.configuration[1]-gripper_closed[1])
        sum_open +=math.fabs(self.configuration[1]-gripper_open[1])
     
        
    if gripper_msg:
      if sum_closed >= 0 and sum_closed < 0.01:
        self.gripper_state = CLOSED
      elif sum_open >= 0 and sum_open < 0.01:
        self.gripper_state = OPEN
      else:
        self.gripper_state = UNKNOWN
      self.gripper_received = True
    

def main():
    action_name = 'gripper_action'
    rospy.init_node(action_name)
    rospy.sleep(0.001)  # wait for time
    gripper_action_server = GripperAction(action_name)

    quit_when_finished = False

      # check for command line arguments, and send goal to action server if required
    myargs = rospy.myargv()[1:]
    if len(myargs):
        goal = TuckArmGoal()
        goal.tuck_gripper = False
        opts, args = getopt.getopt(myargs, 'hqa:', ['quit','action'])
        for opt, action in opts:
          
          if opt in ('-a', '--action'):
              if action in ('open', 'o'):
                  goal.tuck_gripper = False
              elif action in ('close', 'c'):
                  goal.tuck_gripper = True
              else:
                  rospy.logerr('Invalid action for right arm: %s'%action)
                  rospy.signal_shutdown("ERROR")
          if opt in ('--quit', '-q'):
              quit_when_finished = True

          if opt in ('--help', '-h'):
              usage()
          
        gripper_client = actionlib.SimpleActionClient(action_name, TuckArmAction)
        rospy.logdebug('Waiting for action server to start')
        gripper_client.wait_for_server(rospy.Duration(10.0))
        rospy.logdebug('Sending goal to action server')
        gripper_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))

        if quit_when_finished:
          rospy.signal_shutdown("Quitting")
    rospy.spin()

if __name__ == '__main__':
    main()

