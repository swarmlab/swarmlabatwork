#!/usr/bin/env python
import roslib
roslib.load_manifest('slaw_arm_navigation')

import rospy

import math
import getopt

import actionlib
from slaw_arm_navigation.msg import *

from trajectory_msgs.msg import *
from control_msgs.msg import *
from actionlib_msgs.msg import *
from sensor_msgs.msg import *

from std_srvs.srv import *
from geometry_msgs.msg import Point

from slaw_arm_navigation.srv import *

class SimpleIkAction:
    def __init__(self):
        self.arm_received = False
        self.configuration = [0, 0, 0, 0, 0]
        
        self.move_duration = 2.5
        #print self.joint_names
    
        #arm_action_name = rospy.get_param('~arm_joint_trajectory_action', '/arm_1/arm_controller/joint_trajectory_action')
        #self.arm_joint_client = actionlib.SimpleActionClient(arm_action_name, FollowJointTrajectoryAction)
        
        #if not self.arm_joint_client.wait_for_server(rospy.Duration(2)):
	#    rospy.logerr("simple ik solver: arm_joint_client action server did not come up within timelimit")

        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
        self.joints = rospy.get_param('joints')
        self.calc_height_diff_and_max_reach()

        self.arm_rot_offset = rospy.get_param('arm_rot_offset')

        self.joint_names = rospy.get_param("joint_names")
        self.arm_base_link = rospy.get_param("arm_base_link")
        

        self.service = rospy.Service('/arm_1/simple_ik_server', SimpleIkSolver, self.executeCB)
    #self.action_server = actionlib.simple_action_server.SimpleActionServer('/simple_ik_action',TuckArmAction, self.executeCB, False)
    #self.action_server.start()
    
    def calc_height_diff_and_max_reach(self):
        self.max_reach = self.joints['arm_joint_1']['front_offset'] + self.joints['arm_joint_2']['length'] + self.joints['arm_joint_3']['length']
        self.height_diff = self.joints['arm_joint_1']['height'] - self.joints['arm_joint_4']['length'] - self.joints['arm_joint_5']['length']
        #print self.max_reach, self.height_diff

    def executeCB(self, req):
        pointStamped = req.point
        response = SimpleIkSolverResponse()
        if not pointStamped.header.frame_id == self.arm_base_link:
            return response
        offset = 0
        point = pointStamped.point
        if req.position == 'back':
            offset = math.pi
            #point.x *= -1
            #point.y *= -1 
            
        if req.position == 'left':
            offset = math.pi/2
            #tmp = point.x
            #point.x = point.y
            #point.y = -tmp

        if req.position == 'right':
            offset = -math.pi/2
            #tmp = point.x
            #point.x *= -point.y
            #point.y *= tmp

        #print point
        
        conf = self.calc_joints_for_point(point, offset)

        #print conf
        if conf is not None:
            response.joints = tuple(conf)
        return response
        

    def get_min_max_dist_for_height(self, height, start = 0.2, step = 0.001):
        point = Point()
        point.x = start
        point.y = 0
        point.z = height
        while self.calc_joints_for_point(point) is not None:
            point.x -= step
        min_dist = point.x + step

        point.x = start
        while self.calc_joints_for_point(point) is not None:
            point.x += step
        max_dist = point.x - step
        return min_dist, max_dist
            
    def calc_joints_for_point(self,point, offset=0.0):
        height = point.z
        dist = math.sqrt(point.x*point.x + point.y*point.y)
        conf = None
        try:
            conf = self.calc_joints_for_height_and_rect_dist(dist, height)
            angle = math.atan2(point.y, point.x)
            
            conf['arm_joint_1'] -= angle + offset + self.arm_rot_offset
            conf['arm_joint_5'] -= angle

            #conf['arm_joint_5'] = self.calc_joint_5_angle(conf['arm_joint_5'])
        #print conf
        
            if self.check_conf(conf):
                return self.dict_to_conf(conf)

        except Exception, e:
            print "Error:" ,e
            return None
        
        return None
        
        

    def calc_joints_for_height_and_rect_dist(self,dist, height):
        height_diff = height - self.height_diff
        
        dir_dist = math.sqrt(dist*dist + height_diff*height_diff)

        #print dir_dist, height
        return self.calc_joints_for_height_and_dist(dir_dist, height)
    
    
    def calc_joints_for_height_and_dist(self, dist, height):
        if dist > self.max_reach:
            return None
        #adjust for front offset

        offset_joint_2, offset_joint_4 = self.calc_dist_angle(dist,height)
        #print "offsets"
        #print offset_joint_2, offset_joint_4
        
        config = self.calc_joints_for_dist(dist)

        #print config
        
        config['arm_joint_1'] = self.joints['arm_joint_1']['straight']

        config['arm_joint_2'] = self.joints['arm_joint_2']['straight'] + (math.pi/2 - offset_joint_2) - config['arm_joint_2']
        config['arm_joint_3'] = self.joints['arm_joint_3']['straight'] + (math.pi - config['arm_joint_3'])
        config['arm_joint_4'] = self.joints['arm_joint_4']['straight'] + (math.pi - offset_joint_4)- config['arm_joint_4']

        config['arm_joint_5'] = self.joints['arm_joint_5']['straight']

        return config


    def calc_joint_5_angle(self, angle):
        if angle <= self.joints['arm_joint_5']['min']:
            angle += math.pi
        elif angle >= self.joints['arm_joint_5']['max']:
            angle -= math.pi
        return angle
    

    def calc_dist_angle(self, dist, height):
        c = dist - self.joints['arm_joint_1']['front_offset'] 
        a = height - self.height_diff
        alpha = math.asin(a/c)
        beta = math.pi/2.0 - alpha
        return alpha, beta
        
    def calc_joints_for_dist(self, dist):
        if dist == self.max_reach:
            config = {}
            config['arm_joint_2'] = 0
            config['arm_joint_3'] = math.pi
            config['arm_joint_4'] = 0
            return config
            
        a = self.joints['arm_joint_2']['length']
        b = self.joints['arm_joint_3']['length']
        c = dist - self.joints['arm_joint_1']['front_offset'] 


                
        alpha = math.acos((b*b + c*c - a*a) / (2*b*c))
        beta = math.acos((a*a + c*c - b*b) / (2*a*c))
        gamma = math.pi - alpha - beta

        config = {}
        config['arm_joint_2'] = beta
        config['arm_joint_3'] = gamma
        config['arm_joint_4'] = alpha
        return config
        
    
    def dict_to_conf(self, confDict):
        conf = []
        conf.append(confDict['arm_joint_1'])
        conf.append(confDict['arm_joint_2'])
        conf.append(confDict['arm_joint_3'])
        conf.append(confDict['arm_joint_4'])
        conf.append(confDict['arm_joint_5'])
        return conf
    
    def check_conf(self, confDict):
        for name, val in confDict.items():
            if val <= self.joints[name]['min'] or val >= self.joints[name]['max']:
                return False
        return True
    
    def joint_states_callback(self, msg):
        for k in range(len(self.joint_names)):
            for i in range(len(msg.name)):
                if (msg.name[i] == self.joint_names[k]):
                    self.configuration[k] = msg.position[i]
		self.arm_received = True

    def go_straight(self):
        config = {}
        config['arm_joint_1'] = self.joints['arm_joint_1']['straight'] 
        config['arm_joint_2'] = self.joints['arm_joint_2']['straight'] 
        config['arm_joint_3'] = self.joints['arm_joint_3']['straight']
        config['arm_joint_4'] = self.joints['arm_joint_4']['straight'] 
        config['arm_joint_5'] = self.joints['arm_joint_5']['straight']

        conf = self.dict_to_conf(config)
        #print conf
        self.go([conf])


    def max_pick_straight(self):
        config = {}
        config['arm_joint_1'] = self.joints['arm_joint_1']['straight']
        config['arm_joint_2'] = self.joints['arm_joint_2']['straight'] - math.pi/2
        config['arm_joint_3'] = self.joints['arm_joint_3']['straight']
        config['arm_joint_4'] = self.joints['arm_joint_4']['straight'] + math.pi/2
        config['arm_joint_5'] = self.joints['arm_joint_5']['straight']

        conf = self.dict_to_conf(config)
        return conf
    
  
    # def go(self, positions, wait = True):
    #     goal = FollowJointTrajectoryGoal()
    #     goal.trajectory.joint_names = [x for x in self.joint_names]
    #     goal.trajectory.points = []
    #     for p, count in zip(positions, range(0,len(positions)+1)):
    #         goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
    #                                                       velocities = [],
    #                                                       accelerations = [],
    #                                                       time_from_start = rospy.Duration((count+1) * self.move_duration)))
    #     goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)
    # #print goal
    #     if wait:
    #         if not self.arm_joint_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0)):
    #             self.success = False
    #             return
    #         else:
    #             self.arm_joint_client.send_goal(goal)
    #             return

    
if __name__ == "__main__":
	rospy.init_node("joint_trajectory_action")
	rospy.sleep(0.1)
	
	action = SimpleIkAction()

        height = 0.105 - 0.177
        min_dist, max_dist = action.get_min_max_dist_for_height(height)
        print min_dist, max_dist
        point = Point()
        point.z = height

        point.y = -min_dist
        #point.y = -0.05
        test = action.calc_joints_for_point(point, offset=0)
        print "min", test
        point.y = -max_dist

        test = action.calc_joints_for_point(point, offset=0)

        print 'max', test
        rospy.spin()
        

        #dist = 0.347 - 0.04
        #height = -0.05

        #point = Point()
        #point.z = height

        #point.x = 0.25
        #point.y = -0.05
        
        #action.go_straight()
        #print action.max_pick_straight()
        #for i in xrange(10):
        #    test = action.calc_joints_for_point(point, offset=-math.pi/2)
        #     point.y += 0.01
#             print test
#             action.go([test])
#             rospy.sleep(1.)
# #rospy.spin()
