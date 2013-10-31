#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/point_types.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <float.h>

#include <geometry_msgs/Pose2D.h>

#include <math.h>

#define PI 3.14159265

int max_iter_ = 1;
int max_dist_ = 5;
int min_points_ = 100;

std::vector<float> theta;

ros::Subscriber pcl_sub_;
ros::Publisher pcl_pub_;
ros::Publisher pcl_pub_debug_;
ros::Publisher theta_pub_;

float l2pDistance(Eigen::VectorXf model_coeffecients, Eigen::Vector2f p){
  Eigen::Vector2f a(model_coeffecients[0], model_coeffecients[1]);  
  Eigen::Vector2f t(model_coeffecients[3], model_coeffecients[4]);
  Eigen::Vector2f n(1,1);
  return 0;
}

float getWrappedAngle(Eigen::VectorXf model_coeffecients){
  float theta = atan(model_coeffecients[3] / model_coeffecients[4]);// * 180 / PI; //possibly need to turn by 90deg todo also make this nicer...
  if (theta > -PI/4 && theta < PI/4) //i.e. between -45 and + 45 deg
    return theta;
  else
    if (theta >= PI/4 && theta < 3*PI/4 ) //between 45 and 135 
      return theta-PI/2;
    else
      if (theta >= -3*PI/4 && theta < -PI/4 ) //between -45 and -135 
	return theta+PI/2;
      else {
	ROS_ERROR("this should not have happened");
	return 0;
      }
}


float cutPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  pcl::PointCloud<pcl::PointXYZ>::iterator iter=cloud->begin();  
  while (iter != cloud->end()) {
    Eigen::Vector3f vec(iter->x, iter->y, iter->z);
    if (!isnan(vec.norm()) && vec.norm() > max_dist_) {
      cloud->erase(iter);
    }
    else
      iter++;
  }

}

void pcl_cb(sensor_msgs::PointCloud2::ConstPtr cloud_msg){

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud); //this probably copies...
  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>(cloud)); //..double copy unecessary
  cutPCL(newCloud);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (!(newCloud->width < min_points_)) { //newcloud->points.size()
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (newCloud));  
  
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > lineClouds;

    int iter=-1;
    while(++iter<max_iter_ && !(newCloud->width < min_points_)) {
      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);  
      ransac.setDistanceThreshold (.01);
      bool lineFound = ransac.computeModel();
      if (lineFound) {
	std::vector<int> inliers;
	ransac.getInliers(inliers);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*newCloud, inliers, *tempCloud);
	(*finalCloud)+=(*tempCloud);
	lineClouds.push_back(tempCloud);

	Eigen::VectorXf model_coeffecients;
	ransac.getModelCoefficients(model_coeffecients);
	theta[iter] = getWrappedAngle(model_coeffecients);
      
      
	pcl::PointCloud<pcl::PointXYZ>::iterator del=newCloud->begin();
	for (int i=inliers.size()-1; i>=0; i--){
	  newCloud->erase(del+inliers[i]);
	}
      }
      else
	iter = max_iter_;
    }

  
    sensor_msgs::PointCloud2 pub_msg;
  
    pcl::toROSMsg(*finalCloud, pub_msg);
    pub_msg.header.frame_id = "/base_footprint";
    pub_msg.header.stamp = ros::Time::now();
    pcl_pub_.publish(pub_msg);

    float avgTheta = 0;
    for (int i=0; i<max_iter_; i++){
      avgTheta += theta[i];
    }
    avgTheta/=max_iter_;
    geometry_msgs::Pose2D theta_msg;
    //    theta_msg.header = pub_msg.header;
    theta_msg.theta = -avgTheta;
    theta_pub_.publish(theta_msg);
    
    sensor_msgs::PointCloud2 pub_msg_debug;
    pcl::toROSMsg(*newCloud, pub_msg_debug);
    pub_msg_debug.header.frame_id = "/base_footprint";
    pub_msg_debug.header.stamp = pub_msg.header.stamp;
    pcl_pub_debug_.publish(pub_msg_debug);

  }
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ransac_line");
  ros::NodeHandle nh;

  pcl_sub_ = nh.subscribe("/scan_registration/cloud_combined", 1, pcl_cb);
  pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/ransac_cloud",1);
  pcl_pub_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/scan_registration/ransac_cloud_debug",1);
  theta_pub_ = nh.advertise<geometry_msgs::Pose2D>("/scan_registration/theta",1);
  theta.resize(max_iter_);
  
  sleep(1);
  ros::Rate r(10); // 10 hz
  
  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
