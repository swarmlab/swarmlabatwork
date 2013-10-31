#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

double HOLD_OFF_TIME = 0.5;

std::vector<std::string> topics;
std::vector<ros::Subscriber> subscribers;
ros::Publisher twist_pub;
geometry_msgs::Twist twist_mixed;

ros::Time time_last;
int current_level = 0;

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg, int32_t priority) {
    ros::Time time_now = ros::Time::now();
    
    if ( (priority >= current_level) | ((time_now - time_last).toSec() > HOLD_OFF_TIME) ) {
        if (current_level != priority) {
            ROS_DEBUG("New level: %d", priority);
        }
        current_level = priority;
        time_last = ros::Time::now();
        twist_mixed.linear.x = msg->linear.x;
	twist_mixed.linear.y = msg->linear.y;
        twist_mixed.angular.z = msg->angular.z;
        twist_pub.publish(twist_mixed);
        
        time_last = time_now;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "twist_mixer");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    time_last = ros::Time::now();

    XmlRpc::XmlRpcValue list;

    try {
        priv_nh.getParam("hold_off_time", HOLD_OFF_TIME);   
        priv_nh.getParam("twist_mixer_topics", list);
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
          
        for (int32_t i = 0; i < list.size(); ++i) 
        {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            topics.push_back(static_cast<std::string>(list[i]));
        }
    }
    catch (ros::Exception e) {
        ROS_ERROR("Parameter not set: %s", e.what());
        return 1;
    }
    
    for (std::vector<std::string>::size_type i = 0; i < topics.size(); i++) {
        subscribers.push_back(nh.subscribe<geometry_msgs::Twist>(topics[i], 10, boost::bind(twist_cb, _1, i)));
    }

    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_twist_mixed", 10);

    ros::spin();
    return 0;
}
