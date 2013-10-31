#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <slaw_navigation/switchOff.h>

ros::Subscriber pcl_sub_;
ros::Publisher twist_pub_;
ros::ServiceServer serviceOff;

float max_speed_ = 0.11;
float min_speed_ = 0.09;
float scale_ = 0.75;
float max_range_ = 0.75;

bool paused_ = true;

Eigen::Vector3f weight(Eigen::Vector3f in){
  //return Eigen::Vector3f(.1/in.x(), .1/in.y(), 0);
  return in;
}



bool turnOff(slaw_navigation::switchOff::Request &req, slaw_navigation::switchOff::Response &res) {
  ROS_ERROR("TURN %d", req.pause);
  paused_=req.pause;
  res.success = true;
  return true;
}


void cb_cloud(sensor_msgs::PointCloud2::Ptr cloud_msg){
  if (!paused_) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud.begin();
    Eigen::Vector3f forceVector(0, 0, 0);
    int numPoints = 0;

    while (it != cloud.end()){
      Eigen::Vector3f aPoint(it->x, it->y, 0);
      if (!(aPoint.norm()>max_range_)) {
	forceVector -= weight(aPoint);
	numPoints++;
      }
      ++it;
    }

    if (numPoints == 0) {
      //if (max_range_ < 1.5)
      //max_range_ += 0.1;
      forceVector = Eigen::Vector3f(0.1,0,0);
    }
    else
      {
	forceVector/=numPoints;
	forceVector*=scale_;
	if (forceVector.norm() > max_speed_){
	  forceVector.normalize();
	  forceVector *= max_speed_;
	}
	if (forceVector.norm() < min_speed_){
	  forceVector.normalize();
	  forceVector *= min_speed_;
	}
      }
    geometry_msgs::Twist twist_msg;
  
    twist_msg.linear.x = forceVector.x();
    twist_msg.linear.y = forceVector.y();
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;
    twist_pub_.publish(twist_msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ff_recovery");
  ros::NodeHandle nh;

  serviceOff = nh.advertiseService("/scan_registration/switchOffForcefield", turnOff);
  twist_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  pcl_sub_ = nh.subscribe("/scan_registration/cloud_combined", 1, cb_cloud);
  
  ros::spin();
}

