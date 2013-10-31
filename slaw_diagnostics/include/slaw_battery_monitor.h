/*
 * youbot_battery_monitor.h
 *
 *  Created on: Nov 30, 2012
 *      Author: Frederik Hegger, Jan Paulus
 *
 *  adapted for SLAW
 */

#ifndef SLAW_BATTERY_MONITOR_H_
#define SLAW_BATTERY_MONITOR_H_

#include <ros/ros.h>
#include <slaw_diagnostics/BatteryStatus.h>
#include <slaw_diagnostics/SysInfo.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <iostream>
#include <string>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>


#define MIN_VOLTAGE     				18  // Volt
#define MAX_VOLTAGE    					25  // Volt

#define BATTERY_PERCENTAGE_THRESHOLD    30

namespace slaw
{

enum DisplayLine { line2 = 0x02, line3 = 0x03 };
enum VoltageSource { battery1 = 0x04, battery2 = 0x05, powersupply = 0x0c };

class SlawBatteryMonitor
{
public:
	SlawBatteryMonitor(ros::NodeHandle n);
	~SlawBatteryMonitor();
	
	void cb_sysinfo(const slaw_diagnostics::SysInfo::ConstPtr& msg);

	bool connect(std::string port);
	bool disconnect();

	/* returns the voltage in [Volt] */
	double getVoltage(VoltageSource source);

	/* set text on the youBot LCD display */
	bool setYoubotDisplayText(DisplayLine line, std::string text);

	void publishStatusInformation();

private:
	void configureSerialPort();

	bool ros_node_initialized_;
	ros::NodeHandle nh_;
	ros::Publisher pub_battery_status_;
	ros::Publisher pub_diagnostics_;
	ros::Subscriber sub_sysinfo_;

	slaw_diagnostics::BatteryStatus battery_message_;
	diagnostic_msgs::DiagnosticArray diagnostic_array_;
	diagnostic_msgs::DiagnosticStatus diagnostic_state_;

	int serial_file_description_;
	bool is_connected_;
	float pc_power_;
};

} /* namespace youbot */
#endif /* YOUBOT_BATTERY_MONITOR_H_ */
