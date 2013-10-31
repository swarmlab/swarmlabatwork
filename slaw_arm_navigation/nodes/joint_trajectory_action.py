#!/usr/bin/env python
import roslib; roslib.load_manifest('slaw_arm_navigation')
import math
import rospy
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import control_msgs.msg
import numpy as np

arm_up_pose = [0.011, 1.04883, -2.43523, 1.73184, 0.2]
LIMIT_OVER_VOLT = 1


class JointTrajectoryAction:
	
	def __init__(self):
		self.received_state = False
		
		if (not rospy.has_param("joint_trajectory_action/unit")):
			rospy.logerr("No unit given.")
			exit(0)
		
		if (not rospy.has_param("joint_trajectory_action/joints")):
			rospy.logerr("No joints given.")
			exit(0)
		
		self.joint_names = rospy.get_param("joint_trajectory_action/joints")
		rospy.loginfo("Joints: %s", self.joint_names)
		self.configuration = [0 for i in range(len(self.joint_names))]
		self.voltages = [0 for i in range(len(self.joint_names))]
		
		self.unit = rospy.get_param("joint_trajectory_action/unit")
		rospy.loginfo("Unit: %s", self.unit)

		self.limits = rospy.get_param("joint_trajectory_action/limits")

		self.goal_difs = rospy.get_param("joint_trajectory_action/constraints")

		#print self.goal_difs['goal_dif']

		self.max_voltage = rospy.get_param("joint_trajectory_action/max_voltage")
                #print self.limits
		
		# subscriptions
		rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
		
		self.pub = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
		arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/follow_joint_trajectory')
		self.arm_joint_client = actionlib.SimpleActionClient(arm_action_name, control_msgs.msg.FollowJointTrajectoryAction)

		if not self.arm_joint_client.wait_for_server(rospy.Duration(2)):
			rospy.logerr("python joint traj_action: arm_joint_client action server did not come up within timelimit")

		
		self.action = actionlib.SimpleActionServer("joint_trajectory_action", control_msgs.msg.FollowJointTrajectoryAction, self.execute_cb, False)
		self.action.start()

		self.counter = 0
		#how often does the correct reading have to be there
		self.goal_success = rospy.get_param("joint_trajectory_action/goal_success", 5)
	
	def joint_states_callback(self, msg):
		for k in range(len(self.joint_names)):
			for i in range(len(msg.name)):
				if (msg.name[i] == self.joint_names[k]):
					self.configuration[k] = msg.position[i]
					if len(msg.effort)>i:
						self.voltages[k] = abs(msg.effort[i])
						if self.voltages[k] > self.max_voltage:
							rospy.logerr("joint %d over voltage with (%f)", k, self.voltages[k])
		self.received_state = True
		


	def limit_joint(self, joint, value):
		if joint in self.limits:
			val_min = self.limits[joint]['min']
			val_max = self.limits[joint]['max']
			if value <= val_min:
				return val_min + 0.001
			if value >= val_max:
				return val_max - 0.001
		return value

	def is_over_voltage(self):
		for idx,volt in enumerate(self.voltages):
			if volt > self.max_voltage:
				rospy.logerr("joint %d Over Voltage (%f)", idx+1, volt)
				return True
		return False

	# #Beware does not work yet!!!
	# def execute_cb_with_speed(self, goal):
	# 	is_timed_out = False
	# 	is_over_voltage = False
	# 	over_volt_counter = 0
	# 	start = rospy.Time.now()
	# 	extra_time = rospy.Duration(5.0)

	# 	print goal
		
	# 	for i in range(len(goal.trajectory.points)):
	# 		conf = goal.trajectory.points[i].positions

	# 		goal.trajectory.points[i].velocities = [math.pi/8] * len(self.joint_names)
	# 		goal.trajectory.points[i].accelerations = [0.01] * len(self.joint_names)
	# 		print goal		
	# 		self.arm_joint_client.send_goal(goal)
	# 				# wait to reach the goal position
	# 		while (not rospy.is_shutdown()):
	# 			if (self.is_goal_reached(conf, self.configuration)):
	# 				break
	# 			if (self.is_over_voltage()):
	# 				over_volt_counter += 1
	# 				if over_volt_counter > LIMIT_OVER_VOLT:
	# 					is_over_voltage = True
	# 					break
					
	# 			if ((rospy.Time.now() - start) > (goal.trajectory.points[i].time_from_start + extra_time)):
	# 				is_timed_out = True
	# 				break
	# 			rospy.sleep(0.01)

	# 		if (is_over_voltage):
	# 			break
			
				
	# 		if (is_timed_out):
	# 			break
			
	# 	result = control_msgs.msg.FollowJointTrajectoryResult()
	# 	if is_over_voltage:
			
	# 	if (is_timed_out):
	# 		result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
	# 		self.arm_joint_client.cancel_all_goals()
	# 		self.action.set_aborted(result)
	# 	else:
	# 		result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
	# 		self.arm_joint_client.cancel_all_goals()
	# 		self.action.set_succeeded(result)


	def armUpRecover(self):
		arm_up_without_turn = [x for x in arm_up_pose]
		arm_up_without_turn[0] = self.configuration[0]
		arm_up_without_turn[4] = self.configuration[4]

		
		joint_positions = brics_actuator.msg.JointPositions()
		conf = arm_up_without_turn
		
			# transform from ROS to BRICS message
		for j in range(len(self.joint_names)):
			joint_value = brics_actuator.msg.JointValue()
			joint_value.joint_uri = self.joint_names[j]
			joint_value.value = self.limit_joint(self.joint_names[j],conf[j])
			joint_value.unit = self.unit
			joint_positions.positions.append(joint_value)
		self.pub.publish(joint_positions)
		rospy.sleep(1)

	
	
	def execute_cb(self, goal):
		is_timed_out = False
		is_over_voltage = False
		over_volt_counter = 0
		start = rospy.Time.now()
		extra_time = rospy.Duration(5.0)
		#print goal
		for i in range(len(goal.trajectory.points)):
			joint_positions = brics_actuator.msg.JointPositions()
			conf = np.array(goal.trajectory.points[i].positions)
			
			# transform from ROS to BRICS message
			for j in range(len(self.joint_names)):
				joint_value = brics_actuator.msg.JointValue()
				joint_value.joint_uri = self.joint_names[j]
				joint_value.value = self.limit_joint(self.joint_names[j],conf[j])
				conf[j] = joint_value.value
				joint_value.unit = self.unit
				joint_positions.positions.append(joint_value)
			self.pub.publish(joint_positions)
			
			# wait to reach the goal position
			while (not rospy.is_shutdown()):
				if (self.is_goal_reached(conf, self.configuration)):
					rospy.sleep(0.2)
					break
				if (self.is_over_voltage()):
					over_volt_counter += 1
					rospy.logerr("%d, overvolts", over_volt_counter)
					if over_volt_counter > LIMIT_OVER_VOLT:

						is_over_voltage = True
						break
				if ((rospy.Time.now() - start) > (goal.trajectory.points[i].time_from_start + extra_time)):
					is_timed_out = True
					break
				rospy.sleep(0.01)
			
			if (is_timed_out or is_over_voltage):
				break
			
			
		result = control_msgs.msg.FollowJointTrajectoryResult()
		if (is_over_voltage):
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
			rospy.logerr("arm over-voltage")

			self.armUpRecover()
			self.action.set_aborted(result)

			
		
		if (is_timed_out):
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
			rospy.logerr("arm timed_out")

			self.armUpRecover()
			self.action.set_aborted(result)
		else:
			result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
			self.action.set_succeeded(result)


	def is_goal_reached(self, goal, conf):
		for i in range(len(self.joint_names)):
			if (abs(goal[i] - conf[i]) > self.goal_difs['goal_dif']):
				self.counter = 0
				return False
		#rospy.logerr("done")
		self.counter += 1
		if self.counter > self.goal_success:
			self.counter = 0
			return True
		else:
			return False


if __name__ == "__main__":
	rospy.init_node("joint_trajectory_action")
	rospy.sleep(0.5)
	
	action = JointTrajectoryAction()
	
	rospy.spin()
