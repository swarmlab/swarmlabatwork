#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Char.h"

ros::Publisher menue_choice_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "vision_client");
  ros::NodeHandle nh;

  menue_choice_pub = nh.advertise<std_msgs::Char>("/vision/menue_choice",10);

  char choice = 0;

  while(choice != 'q') {

    printf("Usage: openni_demo [templates.yml]\n\n"
           "Place your object on a planar, featureless surface. With the mouse,\n"
           "frame it in the 'color' window and right click to learn a first template.\n"
           "Then press 'l' to enter online learning mode, and move the camera around.\n"
           "When the match score falls between 90-95%% the demo will add a new template.\n\n"
           "Keys:\n"
           "\t h   -- This help page\n"
           "\t l   -- Toggle online learning\n"
           "\t m   -- Toggle printing match result\n"
           "\t t   -- Toggle printing timings\n"
           "\t w   -- Write learned templates to disk\n"
           "\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
           "\t e r d u i -- undocumented...\n"
           "\t q   -- Quit\n\n");


    std::cin >> choice;
    std_msgs::Char msg;
    msg.data = choice;
    menue_choice_pub.publish(msg);
    ros::spinOnce();
    }

  return 0;
}
