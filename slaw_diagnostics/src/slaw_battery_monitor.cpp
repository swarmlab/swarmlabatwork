/*
 * youbot_battery_monitor.cpp
 *
 *  Created on: Nov 30, 2012
 *      Author: Frederik Hegger, Jan Paulus
 *
 *  adapted for SLAW
 */

#include "slaw_battery_monitor.h"

namespace slaw
{

SlawBatteryMonitor::SlawBatteryMonitor(ros::NodeHandle n)
{
	serial_file_description_ = 0;
	is_connected_ = false;
	pc_power_ = -1;

	nh_ = n;
	
    pub_battery_status_ = nh_.advertise<slaw_diagnostics::BatteryStatus>("dashboard/battery_status", 1);
    pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    
	diagnostic_state_.name = "battery: Base";
	diagnostic_state_.values.resize(5);
}

SlawBatteryMonitor::~SlawBatteryMonitor()
{
	char text [15];
        sprintf (text, "Node offline!");
        this->setYoubotDisplayText(line2, text);
	this->disconnect();
	pub_battery_status_.shutdown();
	pub_diagnostics_.shutdown();
}

void SlawBatteryMonitor::cb_sysinfo(const slaw_diagnostics::SysInfo::ConstPtr& msg) {
    pc_power_ = msg->battery_pc.voltage;
    //std::cout << pc_power_ << std::endl;
}

bool SlawBatteryMonitor::connect(std::string port)
{
	serial_file_description_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if(is_connected_)
		return true;

	// could not open connection
	if (serial_file_description_ == -1)
	{
		std::cout << "could not to connect to serial port: " << port << std::endl;
		return false;
	}
	// port is open
	else
	{
		std::cout << "serial port " << port << "is already open" << std::endl;
		fcntl(serial_file_description_, F_SETFL, 0);
	}

	this->configureSerialPort();

	is_connected_ = true;
	std::cout << "connected to serial port: " << port << std::endl;

	return true;
}

void SlawBatteryMonitor::configureSerialPort()
{
  struct termios port_settings; // structure to store the port settings in

  tcgetattr(serial_file_description_, &port_settings);

  cfsetispeed(&port_settings, B0); // set baud rates
  cfsetospeed(&port_settings, B0);

  port_settings.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...

  //port_settings.c_cflag |= CRTSCTS; //Enable Hardware Flow Control
  port_settings.c_cflag &= ~CRTSCTS; //Disable Hardware Flow Control

  port_settings.c_cflag &= ~PARENB; // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;

  port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //Choosing Raw Input
  port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software flow control
  port_settings.c_oflag &= ~OPOST; //Choosing Raw Output
  port_settings.c_cc[VMIN] = 0;
  port_settings.c_cc[VTIME] = 10; /* set raw input, 1 second timeout */

  tcsetattr(serial_file_description_, TCSANOW, &port_settings); // apply the settings to the port
}

bool SlawBatteryMonitor::disconnect()
{
	if(is_connected_)
		close(serial_file_description_);

	is_connected_ = false;
	std::cout << "disconnected from serial port: " << std::endl;

	return true;
}

void SlawBatteryMonitor::publishStatusInformation()
{
	double bat1_voltage = 0.0, bat2_voltage = 0.0, bat_percentage = 0.0, ext_power_voltage = 0.0;

	// write base voltage to lcd display
	char text [15];
	sprintf (text, "PC bat: %.1fV", pc_power_);
	this->setYoubotDisplayText(line2, text);

	// retrieve battery information
	bat1_voltage = this->getVoltage(battery1);
	bat2_voltage = this->getVoltage(battery2);
	bat_percentage = (((bat1_voltage + bat2_voltage) - MIN_VOLTAGE)/(MAX_VOLTAGE - MIN_VOLTAGE))*100;
	ext_power_voltage = this->getVoltage(powersupply);

	diagnostic_state_.level = diagnostic_msgs::DiagnosticStatus::OK;
	diagnostic_state_.message = "OK";

	if(bat_percentage > 100)
		bat_percentage = 100;

	if(bat_percentage <= BATTERY_PERCENTAGE_THRESHOLD)
	{
		diagnostic_state_.level = diagnostic_msgs::DiagnosticStatus::WARN;
		diagnostic_state_.message = "Battery below 10%";

		// call the beep command
		//ret = system("beep");
	}

	if(bat_percentage <= 0)
	{
		bat_percentage = 0;

		diagnostic_state_.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		diagnostic_state_.message = "Battery almost empty";
	}

	// dashboard msg
	battery_message_.voltage = bat1_voltage + bat2_voltage;
	battery_message_.percent = bat_percentage;
	battery_message_.plugged_in = ext_power_voltage > MIN_VOLTAGE ? true : false;


	// diagnostics msg
	diagnostic_state_.values[0].key = "external power connected";
	diagnostic_state_.values[0].value = ext_power_voltage > MIN_VOLTAGE ? "yes" : "no";
	diagnostic_state_.values[1].key = "external power voltage";
	diagnostic_state_.values[1].value = boost::lexical_cast<std::string>(ext_power_voltage);
	diagnostic_state_.values[2].key = "battery percentage";
	diagnostic_state_.values[2].value = boost::lexical_cast<std::string>(bat_percentage);
	diagnostic_state_.values[3].key = "battery 1 voltage";
	diagnostic_state_.values[3].value = boost::lexical_cast<std::string>(bat1_voltage);
	diagnostic_state_.values[4].key = "battery 2 voltage";
	diagnostic_state_.values[4].value = boost::lexical_cast<std::string>(bat2_voltage);

	diagnostic_array_.status.clear();
	diagnostic_array_.status.push_back(diagnostic_state_);


	if(bat_percentage <= BATTERY_PERCENTAGE_THRESHOLD)
        ROS_ERROR_STREAM("Critical battery level on device <<youbot_battery>>: " << bat_percentage << "%%");

	battery_message_.header.stamp = ros::Time::now();

	diagnostic_array_.header.stamp = ros::Time::now();
	diagnostic_array_.status.clear();
	diagnostic_array_.status.push_back(diagnostic_state_);

	pub_battery_status_.publish(battery_message_);
	pub_diagnostics_.publish(diagnostic_array_);
}

bool SlawBatteryMonitor::setYoubotDisplayText(DisplayLine line, std::string text)
{
	const int size = 18;
	int ret = 0;

	// check if text is to long
	if (text.size() > size - 2)
		return false;

	unsigned char send_bytes[size];
	send_bytes[0] = line;

	for (unsigned int i = 0; i < size - 2; i++)
	{
		if (i < text.size())
			send_bytes[i + 1] = text[i];
		else
			send_bytes[i + 1] = ' ';
	}
	send_bytes[size - 1] = 0x0d; //trailing \CR

	ret = write(serial_file_description_, send_bytes, size); //Send data
	//printf("[");
	//for (int i = 1; i < size - 1; i++)
	//	printf("%c", send_bytes[i]);

	//printf("]\n");

	return true;
}

//return the voltage in [Volt]
double SlawBatteryMonitor::getVoltage(VoltageSource source)
{
	unsigned char send_bytes[1];
	send_bytes[0] = source;
	int ret = 0;

	ret = write(serial_file_description_, send_bytes, 1); // Send data

	const int readsize = 20;
	char read_bytes[readsize] =	{ 0 };

	int nobytesread = 0;
	nobytesread = read(serial_file_description_, read_bytes, readsize);
	read_bytes[nobytesread - 1] = 0; // delete the last tow character \CR+\LF
	read_bytes[nobytesread - 2] = 0;

	std::string value(read_bytes);

	return (double) atoi(value.c_str()) / 1000;
}


} /* namespace youbot */
