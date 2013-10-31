#include <vector>
#include "slaw_vision/TargetArray.h"
#include "slaw_vision/Target.h"
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
//#include <tf/message_filter.h> 

ros::Publisher menue_choice_pub;
std::string mapFrame;
std::string tarFrame;
std::vector<tf::StampedTransform> tf_vector;
std::vector<tf::Transform> tar_vector;
std::vector<tf::Transform>::iterator tar_it = tar_vector.begin() + 1;
int counter = 0; 

//todo synchronize messages, add timetamp?, hardcoded to first tar, resize every tick atm..
void tb_cb(slaw_vision::TargetArray tar){
  tf::Transform tar_tf;
  if (tar.targets.size() > 0) {
    tar_tf.setOrigin(tf::Vector3(tar.targets[0].pose.position.x, tar.targets[0].pose.position.y, tar.targets[0].pose.position.z));
    tar_tf.setRotation(tf::Quaternion(tar.targets[0].pose.orientation.x, tar.targets[0].pose.orientation.y, tar.targets[0].pose.orientation.z, tar.targets[0].pose.orientation.w));
    tar_it = tar_vector.begin() + counter;
    tar_vector.insert(tar_it, tar_tf);
    //tar_vector.push_back(tar_tf);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulation_listener");
  ros::NodeHandle nh;

  tf::TransformListener m_tfListener;

  mapFrame = "/arm_link_0";
  tarFrame = "/arm_link_5";
  int vector_size_step = 20000;
  int vector_size_current = vector_size_step;
  tf_vector.resize(vector_size_step);
  tar_vector.resize(vector_size_step);

  sleep(1);

  bool record = true;
  ros::Subscriber tar_sub = nh.subscribe("/vision/targets", 1, tb_cb);  

  ros::Rate r(10);
  //menue_choice_pub = nh.advertise<std_msgs::Char>("/vision/menue_choice",10);
  while (nh.ok()) {
    ros::Time now = ros::Time();
    std::vector<tf::StampedTransform>::iterator it = tf_vector.begin() + 1;

    if(record) {
      tf::StampedTransform myTransform;
      if (++counter>vector_size_current) {
	tf_vector.resize(vector_size_current+vector_size_step);
	//tar_vector.resize(vector_size_current+vector_size_step); //broken because while resizing new stuff is being inserted
      }
      m_tfListener.waitForTransform(mapFrame, tarFrame, now, ros::Duration(0.2));
      m_tfListener.lookupTransform(mapFrame, tarFrame, now, myTransform);
      it = tf_vector.begin()+counter;
      tf_vector.insert(it, myTransform);

      double roll,pitch,yaw;
      myTransform.getBasis().getRPY(roll, pitch, yaw);
      ROS_ERROR("put %f %f %f %d into vec", roll, pitch, yaw, counter);
      if (counter > 4){
	tf_vector.at(counter-3).getBasis().getRPY(roll,pitch,yaw);
	ROS_ERROR("test 3 %f %f %f %d",roll, pitch, yaw, counter-3);

	tf_vector.at(counter-2).getBasis().getRPY(roll,pitch,yaw);
	ROS_ERROR("test 2 %f %f %f %d",roll, pitch, yaw, counter-2);

	tf_vector.at(counter-1).getBasis().getRPY(roll,pitch,yaw);
	ROS_ERROR("test 1 %f %f %f %d",roll, pitch, yaw, counter-1);

      }
    }
  
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
