#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include "tf_conversions/tf_eigen.h"
//#include "eigen_conversions/eigen_msg.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>

#include <math.h>
#include <string>


ros::Publisher pcl_pub;
ros::Publisher pose_pub;
ros::Publisher pose_stamped_pub;
ros::Publisher pcl_tar_pub;
ros::Subscriber forward_scan_sub;
ros::Subscriber backward_scan_sub;
ros::Subscriber forward_tar_scan_sub;
ros::Subscriber backward_tar_scan_sub;
tf::TransformListener *tf_listener;
laser_geometry::LaserProjection projector;
std::string tarFrame = "base_footprint";

pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud2d;
std::vector<PCLPointCloud> pcls;
bool pc_received[4];
ros::Rate* waiter; 


int icp_max_iter = 500;
int icp_max_range = 2;

bool debug = true;

bool publishCloud = true;
bool pclicp = true;
bool publishPoseStamped = false; //for rviz, broken atm
bool publishPose = true;

void transformEigenToTF(const Eigen::Affine3d &k, tf::Transform &t) {
  t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
  t.setBasis(btMatrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
}



void publishMsgs() {
  //  ROS_INFO("pub combined");

  if(pclicp) {
    PCLPointCloud2d::Ptr tarPCL (new PCLPointCloud2d);
    PCLPointCloud2d::Ptr srcPCL (new PCLPointCloud2d);
    PCLPointCloud::const_iterator it = pcls[0].begin(); //dubidu no resize
    pcl::PointXYZ point;
    while(it != pcls[0].end()) {
      point.x = it->x;
      point.y = it->y;
      point.z = 0;
      tarPCL->push_back(point);
      ++it;
    }
    it = pcls[1].begin();
    while(it != pcls[1].end()) {
      point.x = it->x;
      point.y = it->y;
      point.z = 0;
      tarPCL->push_back(point);
      ++it;
    }
    it = pcls[2].begin();
    while(it != pcls[2].end()) {
      point.x = it->x;
      point.y = it->y;
      point.z = 0;
      srcPCL->push_back(point);
      ++it;
    }
    it = pcls[3].begin();
    while(it != pcls[3].end()) {
      point.x = it->x;
      point.y = it->y;
      point.z = 0;
      srcPCL->push_back(point);
      ++it;
    }

    icp.setInputCloud(srcPCL);
    icp.setInputTarget(tarPCL);

    PCLPointCloud2d final;
    icp.align(final);

    geometry_msgs::Pose2D poseMsg;
    //poseMsg.header.stamp = ros::Time::now;
    //poseMsg.header.frame_id = tarFrame;
    
    if (!icp.hasConverged()) { 
      poseMsg.x = 0;
      poseMsg.y = 0;
      poseMsg.theta = 0;
      if (publishPose)
	pose_pub.publish(poseMsg);
      if (publishPoseStamped) {
	geometry_msgs::PoseStamped poseStampedMsg;
	poseStampedMsg.header.frame_id = tarFrame;
	poseStampedMsg.header.stamp = ros::Time::now();
	geometry_msgs::Pose pose;
	pose.position.x=100;
	pose.position.y=100;
	pose.position.z=100;
	geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromYaw(0);
	pose.orientation = quat;
	poseStampedMsg.pose = pose;
	pose_stamped_pub.publish(poseStampedMsg);
      }
    }
    
    Eigen::Matrix4f mf(icp.getFinalTransformation());
    Eigen::Matrix4d md(mf.cast<double>());
    Eigen::Affine3d affine(md);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    //std::cout << icp.getFinalTransformation() << std::endl; 
    //std::cout << mf << std::endl; 
    //std::cout << std::endl << md << std::endl;

    tf::Transform trans;
    transformEigenToTF(affine,trans);
    float x, y, yaw;
    yaw = tf::getYaw(trans.getRotation());
    x = trans.getOrigin()[0];
    y = trans.getOrigin()[1];

    ROS_INFO("x %f y %f yaw %f", x, y, yaw);
    
    poseMsg.x = x;
    poseMsg.y = y;
    poseMsg.theta = yaw;
    if (publishPose)
      pose_pub.publish(poseMsg);
    if (publishPoseStamped) {
      geometry_msgs::PoseStamped poseStampedMsg;
      poseStampedMsg.header.frame_id = tarFrame;
      poseStampedMsg.header.stamp = ros::Time::now();
      geometry_msgs::Pose pose;
      pose.position.x=0;
      pose.position.y=0;
      pose.position.z=0;
      Eigen::Vector2f dir(x,y);
      float angle = acos(dir.dot(Eigen::Vector2f(1,0)));
      geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromYaw(angle);
      pose.orientation = quat;
      poseStampedMsg.pose = pose;
      pose_stamped_pub.publish(poseStampedMsg);
    }
  }
  
  if(publishCloud) {
  
    sensor_msgs::PointCloud2 pcMsg;

    pcl::toROSMsg(pcls[2]+pcls[3], pcMsg);  
    pcMsg.header.frame_id = tarFrame;
    pcMsg.header.stamp = ros::Time::now();
    //pcl_pub.publish(pcMsg);
    pcl_tar_pub.publish(pcMsg);

    //waiter->sleep();
  
    pcl::toROSMsg(pcls[0]+pcls[1], pcMsg);
    pcMsg.header.frame_id = tarFrame;
    pcMsg.header.stamp = ros::Time::now();
    pcl_pub.publish(pcMsg);
  }

  pc_received[0] = false;
  pc_received[1] = false;
  
}

void transformCloud(PCLPointCloud& pc, tf::Transform sensorToWorldTf) {
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  pcl::transformPointCloud(pc, pc, sensorToWorld);
}

PCLPointCloud process_scan(sensor_msgs::LaserScan scan, int num) {
  //  ROS_INFO("Processing %d", num);
  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform(tarFrame, scan.header.frame_id, now, ros::Duration(1));  
  tf_listener->lookupTransform(tarFrame, scan.header.frame_id, now, transform);  
  
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(scan, cloud);
  PCLPointCloud pcl;
  pcl::fromROSMsg(cloud, pcl);
  transformCloud(pcl, transform);

  // PCLPointCloud returnCloud; //unneccesary copy, no time tho
  // returnCloud.resize(pcl.size());
  // PCLPointCloud::const_iterator it = pcl.begin();
  // while(it != pcl.end()) {
  //   returnCloud.push_back(pcl::PointXYZ(it->x, it->y, 0));
  //   ++it;
  // }

  // return returnCloud; 
  return PCLPointCloud(pcl);
}


void incomingScan(sensor_msgs::LaserScan scan, int num) {
  if (!pc_received[num]) {
    pc_received[num] = true;
    pcls[num] = process_scan(scan,num);
  }
}

void frontTarCallback(sensor_msgs::LaserScan scan) {
  //  ROS_INFO("ftc");
  incomingScan(scan, 2);
}

void rearTarCallback(sensor_msgs::LaserScan scan) {
  //  ROS_INFO("rtc");
  incomingScan(scan, 3);
}

void frontCallback(sensor_msgs::LaserScan scan) {
  //  ROS_INFO("fc");
  if (debug)
    if (!pc_received[2])
      frontTarCallback(scan);
  //end debug

  incomingScan(scan, 0);
}

void rearCallback(sensor_msgs::LaserScan scan) {
  //    ROS_INFO("rc");
  if(debug)
    if (!pc_received[3])
      rearTarCallback(scan);
  //end debug

  incomingScan(scan, 1);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_merger");
  ros::NodeHandle nh;

  ros::Rate _waiter(10);
  waiter = &_waiter; 
  
  tf::TransformListener tf_listener_;
  tf_listener = &tf_listener_;

  pcls.resize(4);
  for (int i = 0; i<pcls.size(); i++)
    pc_received[i] = false;

  ros::Rate r(100); // 10 hz

  if(publishCloud){
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/cloud_combined",10);
    //  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/cloud_combined",10);
    pcl_tar_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/tar_cloud_combined",10);
  }
  if(publishPose)
    pose_pub = nh.advertise<geometry_msgs::Pose2D>("/scan_registration/tarDir",10);
  if(publishPoseStamped)
    pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("/scan_registration/tarDirStamped",10);
  forward_scan_sub = nh.subscribe("/base_scan_front", 1, frontCallback);
  backward_scan_sub = nh.subscribe("/base_scan_rear", 1, rearCallback);
  forward_tar_scan_sub = nh.subscribe("/scan_registration/base_tar_scan_front", 1, frontTarCallback);
  backward_tar_scan_sub = nh.subscribe("/scan_registration/base_tar_scan_rear", 1, rearTarCallback);

  
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (icp_max_range);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (icp_max_iter);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  
  sleep(1);
  
  ROS_INFO("Started laser merger");
  
  while(ros::ok()) {
    bool all_good = true;
    int affe = 4;
    if (debug) affe = 2;
    for (int i=0; i<affe; i++)
      if (!pc_received[i])
	all_good = false;

    if (all_good)
      publishMsgs();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
