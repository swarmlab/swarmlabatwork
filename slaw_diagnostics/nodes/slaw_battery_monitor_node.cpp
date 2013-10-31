/*
 * youbot_battery_monitor.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Frederik Hegger, Jan Paulus
 *  
 *  adapted for SLAW
 */

#include "slaw_battery_monitor.h"

using namespace slaw;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "slaw_battery_monitor");
	ros::NodeHandle n;
    
    std::string serial_port;
    
    try {
	ros::NodeHandle private_nh_("~");
        private_nh_.param("serial_port", serial_port, std::string("/dev/youbot/lcd_display"));
    }
    catch (ros::InvalidNameException e) {
        ROS_ERROR("Parameter not set: %s", e.what());
        return 1;
    }
    
    SlawBatteryMonitor* slaw_battery_monitor = new SlawBatteryMonitor(n);

	// if connecting fails, retry every 2 seconds
	do
	{
		std::cout << "try to connect to serial port: " << serial_port << std::endl;
		sleep(2);
	}while(!slaw_battery_monitor->connect(serial_port));

    ros::Subscriber sub_sysinfo = n.subscribe("dashboard/sysinfo", 10, &SlawBatteryMonitor::cb_sysinfo, slaw_battery_monitor);


    ros::Rate r(0.5);
	while(ros::ok())
	{
		ros::spinOnce();
		slaw_battery_monitor->publishStatusInformation();
		r.sleep();
	}

	slaw_battery_monitor->disconnect();

	return (0);
}

