#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <slaw_registration/setRegistrationTar.h>
#include <slaw_navigation/switchOff.h>
#include <slaw_registration/setOffset.h>
//#include <std_srvs/Empty.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"

#include <pcl17/point_types.h>
#include <pcl17/ros/conversions.h>
#include <pcl17_ros/transforms.h>
#include <pcl17/registration/icp.h>

#include <pcl17/registration/correspondence_rejection_var_trimmed.h>
#include <pcl17/registration/correspondence_rejection_trimmed.h>
#include <pcl17/registration/correspondence_rejection_median_distance.h>
//#include <pcl17/registration/correspondence_rejection_poly.h>
#include <pcl17/registration/correspondence_rejection_one_to_one.h>

#include <math.h>
#include <string>

ros::ServiceServer service;
ros::ServiceServer serviceOff;
ros::ServiceServer serviceOffset;
ros::ServiceClient client;
ros::Publisher pcl17_pub;
ros::Publisher pose_pub;
ros::Publisher pose_stamped_pub;
ros::Publisher pcl17_tar_pub;
ros::Subscriber offset_sub;
ros::Subscriber forward_scan_sub;
ros::Subscriber backward_scan_sub;
ros::Subscriber forward_tar_scan_sub;
ros::Subscriber backward_tar_scan_sub;
tf::TransformListener *tf_listener;
laser_geometry::LaserProjection projector;
std::string tarFrame = "base_footprint";

pcl17::IterativeClosestPoint<pcl17::PointXYZ, pcl17::PointXYZ> icp;
typedef pcl17::PointCloud<pcl17::PointXYZ> PCL17PointCloud;
std::vector<PCL17PointCloud> pcl17s;
bool pc_received[4];
bool processed[4];

ros::Rate* waiter; 

boost::mutex pcl17_lock;

int icp_max_iter = 10;
int ransac_iterations = 10;
double x_offset = 0;
double y_offset = 0;
double icp_max_range = 2;
double ransac_threshold = 0.05;
double size_x = 0.50;
double size_y = 0.25;

bool pauseICP = false;
bool debug = false;

pcl17::registration::CorrespondenceRejectorOneToOne::Ptr rej(new pcl17::registration::CorrespondenceRejectorOneToOne);

bool publishCloud = true;
bool pcl17icp = true;
bool publishPoseStamped = false; //for rviz, broken atm
bool publishPose = true;

PCL17PointCloud shiftedPCL(PCL17PointCloud pcl17, double x, double y){
  PCL17PointCloud shiftedPCL;
  PCL17PointCloud::const_iterator it = pcl17.begin();
  while (it != pcl17.end()){
    pcl17::PointXYZ shiftedPoint(it->x-x, it->y-y, it->z);
    shiftedPCL.push_back(shiftedPoint);
    ++it;
  }
  return shiftedPCL;
}


void transformEigenToTF(const Eigen::Affine3d &k, tf::Transform &t) {
  t.setOrigin(tf::Vector3(k.matrix()(0,3), k.matrix()(1,3), k.matrix()(2,3)));
  t.setBasis(btMatrix3x3(k.matrix()(0,0), k.matrix()(0,1),k.matrix()(0,2),k.matrix()(1,0), k.matrix()(1,1),k.matrix()(1,2),k.matrix()(2,0), k.matrix()(2,1),k.matrix()(2,2)));
}

void init() {
  ros::NodeHandle param_nh("~");
  param_nh.param("registration_debug", debug, debug);
  param_nh.param("registration_icp_max_iter", icp_max_iter, icp_max_iter);
  param_nh.param("registration_ransac_max_iter", ransac_iterations, ransac_iterations);
  param_nh.param("registration_icp_max_range", icp_max_range, icp_max_range);
  param_nh.param("registration_ransac_max_range", ransac_threshold, ransac_threshold);
  param_nh.param("registration_publish_cloud", publishCloud, publishCloud);
  param_nh.param("registration_publish_pose", publishPose, publishPose);
  param_nh.param("registration_publish_pose_stamped", publishPoseStamped, publishPoseStamped);
  
  for(int i =0; i<4;i++) {
    pc_received[i]=false;
    processed[i] = false;
  }

}

Eigen::Vector3f getXYYaw(Eigen::Matrix4f mf){
  Eigen::Matrix4d md(mf.cast<double>());
  Eigen::Affine3d affine(md);
  tf::Transform trans;
  transformEigenToTF(affine,trans);
  Eigen::Vector3f xyyaw;
  xyyaw[2] = tf::getYaw(trans.getRotation());
  xyyaw[0] = trans.getOrigin()[0];
  xyyaw[1] = trans.getOrigin()[1];
  return xyyaw;
}


void publishMsgs() {
  if(debug) {
    debug = false;
    pc_received[0] = false;
    pc_received[1] = false;
    processed[0] = false;
    processed[1] = false;
    return;
  }
  
  PCL17PointCloud::Ptr tarPCL17, srcPCL17;
  ros::Time start =  ros::Time::now();

  if(x_offset != 0 || y_offset !=0) {
    pcl17s[2] = shiftedPCL(pcl17s[2], x_offset, y_offset);
    pcl17s[3] = shiftedPCL(pcl17s[3], x_offset, y_offset);
    x_offset = 0;
    y_offset = 0;
  }
        
  { 
    boost::mutex::scoped_lock lock(pcl17_lock);
    tarPCL17 = PCL17PointCloud::Ptr (new PCL17PointCloud(pcl17s[0]+pcl17s[1]));
    srcPCL17 = PCL17PointCloud::Ptr (new PCL17PointCloud(pcl17s[2]+pcl17s[3]));
  }

  //collision check:
 

  if(pcl17icp && !pauseICP) {

    
    PCL17PointCloud final;

    icp.setInputCloud(srcPCL17);
    icp.setInputTarget(tarPCL17);
    icp.align(final);
    
    Eigen::Vector3f xyyaw = getXYYaw(icp.getFinalTransformation());

    // if (xyyaw[0] < 1 && xyyaw[1] < 1) {
    //   icp.addCorrespondenceRejector(rej);
    //   ROS_INFO("num rejectors %d", icp.getCorrespondenceRejectors().size());    
    //   icp.setInputCloud(srcPCL17);
    //   icp.setInputTarget(tarPCL17);
    //   icp.align(final);
    //   icp.removeCorrespondenceRejector(0);
    // }
    
    geometry_msgs::Pose2D poseMsg;
    
    if (!icp.hasConverged()) { 
      if (publishPose){
	poseMsg.x = 0;
	poseMsg.y = 0;
	poseMsg.theta = 0;
	pose_pub.publish(poseMsg);
      }

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
    
    else {
    
      float x, y, yaw;
      x = xyyaw[0];
      y = xyyaw[1];
      yaw = xyyaw[2];
      
      //      std::cout << "icp has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
      ROS_INFO("x %f y %f yaw %f", x, y, yaw);
      if (publishPose) {
	poseMsg.x = x;
	poseMsg.y = y;
	poseMsg.theta = yaw;
	pose_pub.publish(poseMsg);
      }
      
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

    ros::Time end = ros::Time::now();
    //    ROS_INFO("icp duration: %f", (end - start).toSec());
    
  }
  
  if(publishCloud) {

    waiter->sleep();
  
    sensor_msgs::PointCloud2 pcMsg;

    pcl17::toROSMsg(pcl17s[2]+pcl17s[3], pcMsg);  
    pcMsg.header.frame_id = tarFrame;
    pcMsg.header.stamp = ros::Time::now();
    pcl17_tar_pub.publish(pcMsg);

    pcl17::toROSMsg(pcl17s[0]+pcl17s[1], pcMsg);
    pcMsg.header.frame_id = tarFrame;
    pcMsg.header.stamp = ros::Time::now();
    pcl17_pub.publish(pcMsg);
  }
  
  pc_received[0] = false;
  pc_received[1] = false;
  processed[0] = false;
  processed[1] = false;

}

void transformCloud(PCL17PointCloud& pc, tf::Transform sensorToWorldTf) {
  Eigen::Matrix4f sensorToWorld;
  pcl17_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  pcl17::transformPointCloud(pc, pc, sensorToWorld);
}

PCL17PointCloud process_scan(sensor_msgs::LaserScan scan, int num) {
  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  bool tfFound = tf_listener->waitForTransform(tarFrame, scan.header.frame_id, scan.header.stamp, ros::Duration(1));  
  if (tfFound)
    tf_listener->lookupTransform(tarFrame, scan.header.frame_id, scan.header.stamp, transform);  
  
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(scan, cloud);
  PCL17PointCloud pcl17;
  pcl17::fromROSMsg(cloud, pcl17);
  transformCloud(pcl17, transform);
  
  return pcl17;
}


void incomingScan(sensor_msgs::LaserScan scan, int num) {
    boost::mutex::scoped_lock lock(pcl17_lock);

    if (!pc_received[num]) {
      pc_received[num] = true;
 
      pcl17s[num] = process_scan(scan,num);
      processed[num] = true;
    }
}

void frontTarCallback(sensor_msgs::LaserScan scan) {
  incomingScan(scan, 2);
}

void rearTarCallback(sensor_msgs::LaserScan scan) {
  incomingScan(scan, 3);
}

void frontCallback(sensor_msgs::LaserScan scan) {
  if (debug)
    if (!pc_received[2]){
      //bin/      x_offset = 0.2;
      frontTarCallback(scan);
    }
  incomingScan(scan, 0);
}

void rearCallback(sensor_msgs::LaserScan scan) {
  if(debug)
    if (!pc_received[3]){
      rearTarCallback(scan);
    }
  incomingScan(scan, 1);
}

bool turnOff(slaw_navigation::switchOff::Request &req, slaw_navigation::switchOff::Response &res) {
   pauseICP=req.pause;
   slaw_navigation::switchOff srv;
   srv.request.pause = req.pause;
   client.call(srv);
   if (srv.response.success){
     res.success = true;
     return true;
   }
   else{
       res.success = false;
     return false;
   }
}

bool setTar(slaw_registration::setRegistrationTar::Request  &req,
	    slaw_registration::setRegistrationTar::Response &res)
  {
    pc_received[2] = false;
    processed[2] = false;
    pc_received[3] = false;
    processed[3] = false;
    frontTarCallback(req.front);
    rearTarCallback(req.rear);
    res.success = true;
    pauseICP=false;
    ROS_INFO("set tar request received");
    return true;
  }

bool setOffset(slaw_registration::setOffset::Request  &req,
	    slaw_registration::setOffset::Response &res)
  {
    x_offset = req.x;
    y_offset = req.y;
    res.success = true;
    ROS_INFO("shifted tar");
    return true;
  }


int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_merger");
  ros::NodeHandle nh;

  ros::Rate _waiter(10);
  waiter = &_waiter; 

  init();
  
  tf::TransformListener tf_listener_;
  tf_listener = &tf_listener_;

  pcl17s.resize(4);
  for (unsigned int i = 0; i<pcl17s.size(); i++)
    pc_received[i] = false;

  ros::Rate r(10); // 10 hz

  client = nh.serviceClient<slaw_navigation::switchOff>("/scan_registration/switchOffPID");
  service = nh.advertiseService("/scan_registration/setTar", setTar);
  serviceOff = nh.advertiseService("/scan_registration/switchOffRegistration", turnOff);
  serviceOffset = nh.advertiseService("/scan_registration/setOffset", setOffset);

  if(publishCloud){
    pcl17_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/cloud_combined",10);
    pcl17_tar_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/tar_cloud_combined",10);
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
  icp.setRANSACIterations(ransac_iterations);
  icp.setRANSACOutlierRejectionThreshold(ransac_threshold);
  icp.setUseReciprocalCorrespondences(false); 

  //  pcl17::registration::CorrespondenceRejectorOneToOne::Ptr rej(new pcl17::registration::CorrespondenceRejectorOneToOne);
  icp.addCorrespondenceRejector(rej);
    

  ROS_INFO("ICP started with MaxCorrespondenceDistance %f, MaxIterations %d, RANSAC iterations %f, RANSAC outlier threshold %f", icp_max_range, icp_max_iter, icp.getRANSACIterations() ,icp.getRANSACOutlierRejectionThreshold());
  
  sleep(1);
  
  while(ros::ok()) {

    bool scans_received = true;
    int num_sensors = 2;
    for (int i=0; i<num_sensors*2; i++)
      if (!pc_received[i] || !processed[i])
	scans_received = false;

    if (scans_received){
      publishMsgs();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}



//trash
    //initialCorrespondenceDist
   
    // float corr_dist = icp_max_range;
    // bool cont = true;
    // PCL17PointCloud2d final;
    // while(cont) {
    //   ROS_INFO("corr dist %f", corr_dist);
    //   if (corr_dist < 0.01) {
    // 	cont = false;
    // 	corr_dist = 0.05;
    //   }
    //   icp.setMaxCorrespondenceDistance(corr_dist);
    //   icp.setRANSACOutlierRejectionThreshold(corr_dist);
    //   icp.setInputCloud(srcPCL17);
    //   icp.setInputTarget(tarPCL17);
    //   icp.align(final);
    //   if (icp.hasConverged()) {
    // 	Eigen::Vector3f xyyaw = getXYYaw(icp.getFinalTransformation());
    // 	float dist = sqrt(xyyaw[0]*xyyaw[0]+xyyaw[1]*xyyaw[1]);
    // 	corr_dist = dist*(1.5);
    //   }
    // }
   
   
    // float depth = 1;
    // bool done = false;
    // float corr_dist = 1;
    // while(!done && corr_dist > 0.01){
    //   corr_dist = icp_max_range/(depth*depth);
    //   //      ROS_INFO("min correspondence distance = %f",corr_dist);
    //   icp.setMaxCorrespondenceDistance(corr_dist);
    //   icp.setRANSACOutlierRejectionThreshold(corr_dist);
    //   icp.setInputCloud(srcPCL17);
    //   icp.setInputTarget(tarPCL17);
    //   PCL17PointCloud2d final;
    //   icp.align(final);
    //   if (icp.hasConverged()) {
    // 	//	ROS_INFO("icp converged");
    // 	depth++;
    //   }
    //   else{
    // 	//	ROS_INFO("icp not converged");
    // 	done = true;
    //   }
    // }
    // corr_dist = icp_max_range/(depth*depth);
    // ROS_INFO("min correspondence distance = %f",(corr_dist));
    // icp.setMaxCorrespondenceDistance(corr_dist);
    // icp.setRANSACOutlierRejectionThreshold(corr_dist);
    // icp.setInputCloud(srcPCL17);
    // icp.setInputTarget(tarPCL17);
    // PCL17PointCloud2d final;
    // icp.align(final);



    // float dist = sqrt(xyyaw[0]*xyyaw[0]+xyyaw[1]*xyyaw[1]+xyyaw[2]*xyyaw[2]);
    // ROS_INFO("dist %f", dist);
    // if(dist < ransac_threshold) {
    //   icp.setRANSACIterations(ransac_iterations);
    //   icp.align(final);
    // }

    // Eigen::Vector3f xyyaw;
    // float corr_dist = icp_max_range;
    // bool cont = true;
    // PCL17PointCloud final;
    // while(cont) {
    //   ROS_INFO("corr dist %f", corr_dist);
    //   if (corr_dist < 0.01) {
    // 	cont = false;
    // 	corr_dist = 0.05;
    //   }
    //   icp.setMaxCorrespondenceDistance(corr_dist);
    //   icp.setRANSACOutlierRejectionThreshold(corr_dist);
    //   icp.setInputCloud(srcPCL17);
    //   icp.setInputTarget(tarPCL17);
    //   icp.align(final);
    //   if (icp.hasConverged()) {
    // 	xyyaw = getXYYaw(icp.getFinalTransformation());
    // 	float dist = sqrt(xyyaw[0]*xyyaw[0]+xyyaw[1]*xyyaw[1]);
    // 	corr_dist = dist*(1.5);
    //   }
    // }
