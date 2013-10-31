/******************************************************************************
* mostly copied from youbot_registration
******************************************************************************/
#define PI 3.1415926


#include <ros/ros.h>
#include <tf/transform_listener.h>

// #include <std_srvs/Empty.h>

// messages
#include <brics_actuator/JointPositions.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <cv.h>

#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/plane_angle.hpp>

#define NB_ARM_JOINTS 5

ros::Publisher armPositionsPublisher;
ros::Publisher marker_pub;
ros::Publisher cameramount_pub;

ros::Subscriber image_sub;
ros::Subscriber pcl_sub;

tf::TransformListener *tf_listener;
ros::Timer timer_move;
ros::Timer timer_send;

cv::Point pixelPoint;
pcl::PointCloud<pcl::PointXYZ> cloud;
cv::Mat image;

bool fresh_cloud = false;
bool fresh_image = false;

bool firstrun = true;

int number_of_points;

// for registration
pcl::PointCloud<pcl::PointXYZ> world;
pcl::PointCloud<pcl::PointXYZ> target;

std::string world_frame_id("base_link");
std::string marker_frame_id("pattern_center_circle");
std::string target_frame_id("camera_link");

tf::Transform transform;


const double right_positions[][NB_ARM_JOINTS] =  {
  {1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 2.2230750324694593},
  {1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 1.8230750324694593},
  {1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 2.6230750324694593},

  {1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 2.154247604685531},
  {1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 2.454247604685531},
  {1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 1.754247604685531},

  {1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 2.267721046377517},
  {1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 2.667721046377517},
  {1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 1.867721046377517},

  {2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 2.707875876258988},
  {2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 3.107875876258988},
  {2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 2.307875876258988},

  //straight up for safety reasons
  {2.1867398018357336, 1.04883, -2.43523, 1.73184, 0.2},
  
};


const double up_jointPosition[NB_ARM_JOINTS] = {2.9, 1.04883, -2.43523, 1.73184, 0.2};


const double center_positions[][NB_ARM_JOINTS] =  {
  {0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 2.2230750324694593},
  {0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 1.8230750324694593},
  {0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 2.6230750324694593},


  //straight up for safety reasons
  {0.4175987450321748, 1.04883, -2.43523, 1.73184, 0.2},
  {5.8-0.4175987450321748, 1.04883, -2.43523, 1.73184, 0.2},

  
  {5.8-0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 - 2.2230750324694593},
  {5.8-0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 - 1.8230750324694593},
  {5.8-0.8175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 - 2.6230750324694593},

  //straight up for safety reasons
  {5.8-2.1867398018357336, 1.04883, -2.43523, 1.73184, 0.2},


  {2.4203117556561735, 0.25342180738957665, -0.2803086045165493, 1.786460025296258, 0.08933627559996891},

  {2.6203117556561735, 0.25342180738957665, -0.2803086045165493, 1.786460025296258, 0.08933627559996891},
  {2.9203117556561735, 0.25342180738957665, -0.2803086045165493, 1.786460025296258, 0.08933627559996891},
  {3.2203117556561735, 0.25342180738957665, -0.2803086045165493, 1.786460025296258, 0.08933627559996891},

  {3.5203117556561735, 0.25342180738957665, -0.2803086045165493, 1.786460025296258, 0.08933627559996891},

  //straight up
  {3.5230750324694593, 1.04883, -2.43523, 1.73184, 0.2},

  
};

const double left_positions[][NB_ARM_JOINTS] =  {
  {5.8-1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 - 2.2230750324694593},
  {5.8-1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 -1.8230750324694593},
  {5.8-1.2175987450321748, 0.02445810434477438, -3.2943997202604005, 3.2022342413608467, 5.64 -2.6230750324694593},

  {5.8-1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 5.64 -2.154247604685531},
  {5.8-1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 5.64 -2.454247604685531},
  {5.8-1.7577714357196044, 0.10852591544997048, -2.8059291865169915, 2.1075661927202174, 5.64 -1.754247604685531},

  {5.8-1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 5.64 -2.267721046377517},
  {5.8-1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 5.64 -2.667721046377517},
  {5.8-1.8705566258249227, 0.0010774051728657302, -3.519934656861612, 2.6265484495364806, 5.64 -1.867721046377517},

  {5.8-2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 5.64 -2.707875876258988},
  {5.8-2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 5.64 -3.107875876258988},
  {5.8-2.1867398018357336, 0.0010774051728657302, -3.884909183392406, 2.7728979946315264, 5.64 -2.307875876258988},

  //straight up for safety reasons
  {5.8-2.1867398018357336, 1.04883, -2.43523, 1.73184, 0.2},

  
};



bool detectCirclesGrid()
{
  if (image.size().width) {
    cv::Size patternsize(7,7); //number of centers
    cv::Mat imgresize;
    cv::resize(image,imgresize,cv::Size(image.size().width*2,image.size().height*2));
    cv::Mat gray(imgresize.size(), CV_8U);
    cv::cvtColor(imgresize, gray, CV_BGRA2GRAY);
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers

    bool patternfound =  false;
    for (int i =0; i<10;i++) {
      if (findCirclesGrid(gray, patternsize, centers))
	{
	  patternfound = true;
	  break;
	}
    }
    if (patternfound) {
      pixelPoint.x = centers[24].x/2; // Divided by 2 because the image is enlarged
      pixelPoint.y = centers[24].y/2;
      return true;
    }
    ROS_WARN("FindCircles failed");
  }
  else {
    ROS_WARN("Image is empty");
  }
  return false;
}

void imageReceivedCallback(const sensor_msgs::Image::ConstPtr &img)
{
  cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvShare(img);
  cvtColor(ptr->image.clone(), image, CV_BGR2BGRA);
  fresh_image = true;
}

void pclReceivedCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_pcl)
{
  pcl::fromROSMsg(*ros_pcl,cloud);
  fresh_cloud = true;

}

bool save_target_point()
{
  pcl::PointXYZ point = cloud(pixelPoint.x,pixelPoint.y);

  if (!isnan(point.z) && !isnan(point.y) && !isnan(point.x)) {
    geometry_msgs::PointStamped tempPoint;
    geometry_msgs::PointStamped outPoint;
    tempPoint.point.x = point.x;
    tempPoint.point.y = point.y;
    tempPoint.point.z = point.z;
    tempPoint.header.frame_id = cloud.header.frame_id;
    tf_listener->transformPoint(target_frame_id, tempPoint, outPoint);

    //targetpts_pub.publish(outPoint);
    target.push_back(pcl::PointXYZ(outPoint.point.x, outPoint.point.y, outPoint.point.z));

    //Send Marker

    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_point";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
    return true;
  }
  else
    return false;
}

void save_world_point()
{
  tf::StampedTransform transform;
  tf_listener->lookupTransform(world_frame_id, marker_frame_id,
			       ros::Time(0),transform);

  geometry_msgs::PointStamped world_point;
  world_point.point.x = transform.getOrigin().x();
  world_point.point.y = transform.getOrigin().y();
  world_point.point.z = transform.getOrigin().z();
  world_point.header.frame_id = world_frame_id;

  // worldpts_pub.publish(world_point);
  world.push_back(pcl::PointXYZ(world_point.point.x, world_point.point.y, world_point.point.z));
}

void send_cameramount_position(const std::string position) {
  std_msgs::String msg;
  msg.data = position;
  cameramount_pub.publish(msg);
}

void send_arm_position(const double joint_positions[])
{
  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(NB_ARM_JOINTS);
		
  for (int j = 0; j < NB_ARM_JOINTS; j++)
    {
      std::stringstream jointName;
      jointName << "arm_joint_" << (j + 1);
      armJointPositions[j].joint_uri = jointName.str();
      armJointPositions[j].value = joint_positions[j]; // predefined_jointPositions[position][j];
      armJointPositions[j].unit = boost::units::to_string(boost::units::si::radians);
    }

  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);
  ROS_INFO("Joint positions published.");
}

bool compute_matrix()
{
  if ((target.size() > 2) && (world.size() == target.size())) {
    Eigen::Matrix4f trMatrix;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(target, world, trMatrix);

    transform = tf::Transform(btMatrix3x3(trMatrix(0, 0), trMatrix(0, 1), trMatrix(0, 2),
					  trMatrix(1, 0), trMatrix(1, 1), trMatrix(1, 2),
					  trMatrix(2, 0), trMatrix(2, 1), trMatrix(2, 2)),
			      btVector3(trMatrix(0, 3), trMatrix(1, 3), trMatrix(2, 3)));

    std::cout << "\t\ttranslation:\n"
	      << "\t\t\tx: " << transform.getOrigin().x() << "\n"
	      << "\t\t\ty: " << transform.getOrigin().y() << "\n"
	      << "\t\t\tz: " << transform.getOrigin().z() << "\n"
	      << "\t\trotation:\n"
	      << "\t\t\tx: " << transform.getRotation().x() << "\n"
	      << "\t\t\ty: " << transform.getRotation().y() << "\n"
	      << "\t\t\tz: " << transform.getRotation().z() << "\n"
	      << "\t\t\tw: " << transform.getRotation().w() << "\n";

    return true;
  } else {
    return false;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_kinect_registration");
  ros::NodeHandle n;

  image_sub = n.subscribe("/camera/rgb/image_color",1,&imageReceivedCallback);
  pcl_sub = n.subscribe("/camera/depth_registered/points",1,&pclReceivedCallback);

  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("targetPoint", 1);
  cameramount_pub = n.advertise<std_msgs::String>("/cameramount/position", 1);

  tf_listener = new tf::TransformListener(n);
  ros::Duration timeout(3);
  timeout.sleep();
  timeout.sleep();


  const std::string positions_str[] = {"right", "left", "center"};
  
  for (int pos = 0; pos < 3; pos++) {

    ROS_INFO("Calibrating position %d.", pos);
    send_cameramount_position(positions_str[pos]);
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    timeout.sleep();

    const double (*positions)[NB_ARM_JOINTS];
    
    int len_positions;

    if (pos == 0) {
      positions = right_positions;
      len_positions = sizeof( right_positions ) / (sizeof( right_positions[0][0]) * NB_ARM_JOINTS);

    } else if (pos == 1) {
      positions = left_positions;
      len_positions = sizeof( left_positions ) / (sizeof( left_positions[0][0]) * NB_ARM_JOINTS);

    } else {
      positions = center_positions;
      len_positions = sizeof( center_positions ) / (sizeof( center_positions[0][0]) * NB_ARM_JOINTS);

    }
    ROS_INFO("Number of arm positions %f.", positions[0][0]);


    ROS_INFO("Number of arm positions %d.", len_positions);
    for (int i = 0; i < len_positions; i++) {
      ROS_INFO("pos %d", i);
      send_arm_position(positions[i]);
      timeout.sleep();
    
      // we think arm has reached target position
    
      // wait for fresh point cloud
      fresh_cloud = false;
      fresh_image = false;

      while (!fresh_cloud || !fresh_image) {
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ros::spinOnce();
	ROS_INFO("Waiting for the cloud or image ...");
	timeout.sleep();
      }

      ROS_INFO("Got a cloud and image");

      // image processing
      if (detectCirclesGrid()) {
	ROS_INFO("Marker detected.");
	if (save_target_point()) {
	  ROS_INFO("Target point saved.");
	  save_world_point();
	  ROS_INFO("World point saved.");
	} else {
	  ROS_ERROR("Could not compute target point.");
	}
      } else {
	ROS_ERROR("Marker not detected.");
      }
    }

    // compute transform
    std::cout << "\n\t" << positions_str[pos].c_str() << ":\n";

    if (compute_matrix())
      ROS_INFO("Done!");
    else
      ROS_ERROR("Could not compute transform!");

    // clear points
    target.clear();
    world.clear();

    //bring arm up for less collisions

    
    send_arm_position(up_jointPosition);
    timeout.sleep();
    send_arm_position(up_jointPosition);
    timeout.sleep();


  }
      

  return 0;
}
