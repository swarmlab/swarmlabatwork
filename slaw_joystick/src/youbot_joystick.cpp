#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_listener.h"       

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

using namespace std;

ros::Publisher armPositionsPublisher;
ros::Publisher gripperPositionPublisher;
ros::Publisher baseVelocityPublisher;
ros::Publisher armPosePublisher;

ros::ServiceClient arm_service_client;
ros::ServiceClient base_service_client;

static const float twistScale = 0.5;
static const int numberOfArmJoints = 5;
static const int numberOfGripperJoints = 2;
static const float armScale = 0.25;

bool zeroSent;
float armSetpoints[numberOfArmJoints];
float gripperSetpoints[numberOfGripperJoints];
geometry_msgs::PoseStamped armPose;
ros::Time ikStamp;
ros::Duration ikInterval(0.01);

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg) {

  //"init"
  float min,max,epsilon;
  brics_actuator::JointPositions command;
  vector <brics_actuator::JointValue> armJointPositions;
  vector <brics_actuator::JointValue> gripperJointPositions;

  armJointPositions.resize(numberOfArmJoints); //TODO:change that
  gripperJointPositions.resize(numberOfGripperJoints);


  //arm
  std::stringstream jointName;
  if( (joy_msg->buttons[PS3_AXIS_BUTTON_REAR_RIGHT_1] >0.5 && joy_msg->buttons[PS3_AXIS_BUTTON_REAR_LEFT_1] < 0.5) || joy_msg->buttons[PS3_BUTTON_ACTION_TRIANGLE] ) {

    float armDeltas[numberOfArmJoints];
    
    armDeltas[0] = -joy_msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
    armDeltas[1] = joy_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    armDeltas[2] = -joy_msg->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
    armDeltas[3] = 0;
    armDeltas[3] += joy_msg->axes[PS3_AXIS_BUTTON_REAR_RIGHT_2];
    armDeltas[3] -= joy_msg->axes[PS3_AXIS_BUTTON_REAR_LEFT_2];
    armDeltas[4] = joy_msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    
    for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i + 1);
    
      armJointPositions[i].joint_uri = jointName.str();
    
      epsilon = 0.000001;
      switch (i) {
      case 0: 
	min = 0.0100692;
	max = 5.84014;
	break;

      case 1: 
	min = 0.0100692;
	max = 2.61799;
	break;

      case 2:
	min = -5.02655;
	max = -0.015708;
	break;

      case 3:
	min = 0.0221239;
	max = 3.4292;
	break;

      case 4:
	min = 0.110619;
	max = 5.64159;
	break;
      }

      armSetpoints[i] += ( armDeltas[i] * (1/(max-min)) ) * (armScale);

      if (armSetpoints[i] <= min) 
	armSetpoints[i]= min + epsilon;
      else if(armSetpoints[i] >= max)
	armSetpoints[i] = max - epsilon; 

      if (joy_msg->buttons[PS3_BUTTON_ACTION_TRIANGLE]) {
	if (i != 2)
	  armSetpoints[i] = min + epsilon;
	else 
	  if (i == 2)
	    armSetpoints[i] = max - epsilon;
      }

      armJointPositions[i].value = armSetpoints[i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    }

    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);
    
  }
    
  //gripper
  if(joy_msg->buttons[PS3_BUTTON_REAR_RIGHT_2] == 1) {
    
    min = 0;
    max = 0.0115;
    gripperSetpoints[0] += (joy_msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] / (1/(max-min)) ) * armScale/10; 
    gripperSetpoints[1] += (joy_msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] / (1/(max-min)) ) * armScale/10;

    for (int i=0; i<numberOfGripperJoints; i++) {
      if (gripperSetpoints[i] < min) 
	gripperSetpoints[i] = min+epsilon;
      else
	if(gripperSetpoints[i] > max)
	  gripperSetpoints[i] = max - epsilon;
      
      if (i == 0)
	gripperJointPositions[i].joint_uri = "gripper_finger_joint_l";
      else
	gripperJointPositions[i].joint_uri = "gripper_finger_joint_r";
      
      gripperJointPositions[i].value = gripperSetpoints[i];
      gripperJointPositions[i].unit = boost::units::to_string(boost::units::si::meter);

    }
      
    command.positions = gripperJointPositions;
    gripperPositionPublisher.publish(command);
  }

  //inverse kinematics
    if(joy_msg->buttons[PS3_BUTTON_REAR_LEFT_2] == 1) {
      ros::Duration temp_time =  ros::Time::now() - ikStamp;
      if (temp_time > ikInterval) {
	ikStamp = ros::Time::now();
	armPose.pose.position.y = armPose.pose.position.y + joy_msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * armScale/100;
	armPose.pose.position.x = armPose.pose.position.x + joy_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS] * armScale/100;
	if (joy_msg->axes[PS3_AXIS_BUTTON_CROSS_UP])
	  armPose.pose.position.z = armPose.pose.position.z + armScale/100;
	if (joy_msg->axes[PS3_AXIS_BUTTON_CROSS_DOWN])
	  armPose.pose.position.z = armPose.pose.position.z - armScale/100;
     
	//      Orientation
	tf::Quaternion temp2 = tf::createQuaternionFromRPY(0, 3.14159265/2, 0);
	geometry_msgs::Quaternion temp;
	tf::quaternionTFToMsg(temp2, temp);
	armPose.pose.orientation.x = temp.x;
	armPose.pose.orientation.y = temp.y;
	armPose.pose.orientation.z = temp.z;
	armPose.pose.orientation.w = temp.w;
    

	armPose.header.stamp = ros::Time::now();
	armPosePublisher.publish(armPose);

	ROS_WARN("sent");
      }
      else
	ROS_WARN("waiting..");
    }


  //base vel
  geometry_msgs::Twist twist;
  if(joy_msg->buttons[PS3_AXIS_BUTTON_REAR_LEFT_1] == 1 && joy_msg->buttons[PS3_AXIS_BUTTON_REAR_RIGHT_1] == 0 ) {

    twist.linear.y = joy_msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
    twist.linear.y *= twistScale;
    twist.linear.x = joy_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
    twist.linear.x *= twistScale;
    twist.linear.z = 0;
            
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = joy_msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
    twist.angular.z *= twistScale;
    
    if(zeroSent)
      zeroSent = false;
    
    baseVelocityPublisher.publish(twist);
  }
  
  if(!zeroSent && joy_msg->buttons[PS3_AXIS_BUTTON_REAR_LEFT_1] == 0) {
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    baseVelocityPublisher.publish(twist);
    zeroSent = true;
  }

  if (joy_msg->buttons[PS3_BUTTON_ACTION_CROSS]) {
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    baseVelocityPublisher.publish(twist);
    sleep(1);
    std_srvs::Empty emp;
    base_service_client.call(emp);
    arm_service_client.call(emp);
  }



}



  int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_joy_test");
    ros::NodeHandle n;

    armPositionsPublisher = n.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);
    gripperPositionPublisher = n.advertise<brics_actuator::JointPositions> ("arm_1/gripper_controller/position_command", 1);
    baseVelocityPublisher = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    armPosePublisher = n.advertise<geometry_msgs::PoseStamped>("arm_pose_in", 1);

    base_service_client = n.serviceClient<std_srvs::Empty>("/base/switchOffMotors");
    arm_service_client = n.serviceClient<std_srvs::Empty>("/arm_1/switchOffMotors");

    ros::Subscriber sub = n.subscribe("joy", 100, joy_cb);


    armPose.pose.position.x = 0.024 + 0.033 + 0.4;
    armPose.pose.position.y = 0;
    armPose.pose.position.z = 0.115;
    
    ikStamp = ros::Time::now();

    ros::spin();

    // ros::Rate rate(20); //Hz
    // while(n.ok()) {
    //   ros::spinOnce();
    //   rate.sleep();
    // }

    return 0;
  }
  /* EOF */
