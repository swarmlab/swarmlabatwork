#!/usr/bin/env python
import roslib; roslib.load_manifest("slaw_diagnostics")
import rospy
from slaw_diagnostics.msg import SysInfo, SystemStatus, NetworkStatus, BatteryStatus
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from pythonwifi.iwlibs import Wireless
import psutil
import PyNUT
import socket
import numpy
import os
import re

class SystemInfo():

    BAT_PERC_WARN = 40
    BAT_PERC_ERROR = 20
    #BAT_VOLT_WARN = 20
    #BAT_VOLT_ERROR = 18
    CPU_TEMP_WARN = 80
    CPU_TEMP_ERROR = 90
    CPU_USAGE_WARN = 90
    CPU_USAGE_ERROR = 95

    def __init__(self):
        rospy.init_node('sysinfo')
        pub = rospy.Publisher('dashboard/sysinfo', SysInfo)
        pub_diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray)
        rospy.Subscriber('dashboard/battery_status', BatteryStatus, self.cb_base_bat)
        
        wifi_name = 'wlan0'
        if rospy.has_param('~wifi_name'):
            wifi_name = rospy.get_param('~wifi_name')
        self.wifi = Wireless(wifi_name)
        
        self.eth_name = 'eth0'
        if rospy.has_param('~eth_name'):
            self.eth_name = rospy.get_param('~eth_name')
            
        self.base_bat_voltage = -1
        self.base_bat_watt = -1
        self.base_bat_percent = -1
        self.base_bat_temp = -1
        self.base_bat_plugged_in = False
        
        #self.stat_bat_base = []
        self.stat_bat_pc = []
        self.stat_network = []
        self.stat_system = []
        
        info_msg = SysInfo()
        info_msg.hostname = socket.gethostname()
        
        diag_msg = DiagnosticArray()
        
        r = rospy.Rate(1)
        
        while not rospy.is_shutdown():
            info_msg.header.stamp = rospy.Time.now()
            info_msg.system = self.system_status()
            info_msg.network = self.network_status(self.eth_name)
            info_msg.battery_pc = self.battery_pc_status()
            info_msg.battery_base = self.battery_base_status()
            pub.publish(info_msg)
            
            diag_msg.header.stamp = rospy.Time.now()
            #diag_msg.status.append(self.stat_bat_base)
            diag_msg.status.append(self.stat_bat_pc)
            diag_msg.status.append(self.stat_network)
            diag_msg.status.append(self.stat_system)
            pub_diagnostics.publish(diag_msg)
            
            r.sleep()


    def network_status(self, eth_name):
        msg = NetworkStatus()
        msg.wifi_signallevel = -1.0
        try:
            msg.wifi_signallevel = self.wifi.getStatistics()[1].getSignallevel()
        except:
            pass
        
        fn = "/sys/class/net/%s/operstate"%(self.eth_name)
        try:
            f = open( fn )
            state = f.read().strip()
            msg.ethernet_connected = True
        except:
            rospy.logerr("Can't open file %s"%fn)
            msg.ethernet_connected = False
        
        self.stat_network = DiagnosticStatus(name="computer: Network",level=DiagnosticStatus.OK,message="OK")
        self.stat_network.values = [KeyValue("WiFi signal strength (db)",str(msg.wifi_signallevel)),
                                    KeyValue("Ethernet connected",str(msg.ethernet_connected))]
        
        return msg


    def system_status(self):
        msg = SystemStatus()
        msg.cpu_usage_average = psutil.cpu_percent(interval=0.0)
        msg.cpu_usage_detail = psutil.cpu_percent(interval=0.0, percpu=True)
        msg.mem_usage = psutil.phymem_usage()[3]
        
        temps = []
        res = os.popen("sensors | grep Core")
        for line in res.readlines():
            temps.append(float(re.search('\+(.*?)\W\WC', line).group(1)))
        msg.cpu_temp_detail = temps
        msg.cpu_temp_average = numpy.mean(temps)
        
        self.stat_system = DiagnosticStatus(name="computer: System",level=DiagnosticStatus.OK,message="OK")
        self.stat_system.values = [ KeyValue("CPU usage",str(msg.cpu_usage_average)),
                                    KeyValue("CPU temp (C)",str(msg.cpu_temp_average)),
                                    KeyValue("Memory usage",str(msg.mem_usage))]
        
        if msg.cpu_temp_average > SystemInfo.CPU_TEMP_ERROR:
            self.stat_system.level = DiagnosticStatus.ERROR
            self.stat_system.message = "CPU overheating"
        elif msg.cpu_temp_average > SystemInfo.CPU_TEMP_WARN:
            self.stat_system.level = DiagnosticStatus.WARNING
            self.stat_system.message = "CPU overheating"    
        elif msg.cpu_usage_average > SystemInfo.CPU_USAGE_ERROR:
            self.stat_system.level = DiagnosticStatus.ERROR
            self.stat_system.message = "High CPU load"
        elif msg.cpu_usage_average > SystemInfo.CPU_USAGE_WARN:
            self.stat_system.level = DiagnosticStatus.WARN
            self.stat_system.message = "High CPU load"
                
        return msg   


    def battery_pc_status(self):
        msg = BatteryStatus()
        
        msg.percent = -1
        msg.plugged_in = False
        msg.voltage = -1
        msg.watt = -1
        msg.temp = -1
        
        try:
            ups = PyNUT.PyNUTClient(host='localhost')
            bat = ups.GetUPSVars(ups='openups')
            msg.percent = float(bat['battery.charge'])
            msg.plugged_in = (float(bat['input.voltage']) > 10.0)
            msg.voltage = float(bat['battery.voltage'])
            if msg.plugged_in:
                msg.watt = float(bat['input.voltage']) * float(bat['output.current'])
            else:
                msg.watt = float(bat['output.voltage']) * float(bat['output.current'])
	    msg.temp = float(bat['battery.temperature'])        
        except:
            rospy.logerr("Cannot connect to power board.")
            
        self.stat_bat_pc = DiagnosticStatus(name="battery: PC",level=DiagnosticStatus.OK,message="OK")
        self.stat_bat_pc.values = [ KeyValue("Voltage (V)",str(msg.voltage)),
                                    KeyValue("Percentage",str(msg.percent)),
                                    KeyValue("Power (W)",str(msg.watt)),
                                    KeyValue("Temperature (C)",str(msg.temp)),
                                    KeyValue("Charging",str(msg.plugged_in))]
        
        if msg.percent < SystemInfo.BAT_PERC_ERROR:
            self.stat_bat_pc.level = DiagnosticStatus.ERROR
            self.stat_bat_pc.message = "Battery almost empty"
        elif msg.percent < SystemInfo.BAT_PERC_WARN:
            self.stat_bat_pc.level = DiagnosticStatus.WARN
            self.stat_bat_pc.message = "Battery almost empty"
        
        return msg

    def battery_base_status(self):
        msg = BatteryStatus()
  
        msg.percent = self.base_bat_percent
        msg.plugged_in = self.base_bat_plugged_in
        msg.voltage = self.base_bat_voltage
        msg.watt = self.base_bat_watt
        msg.temp = self.base_bat_temp
        
        #self.stat_bat_base = DiagnosticStatus(name="Base Battery",level=DiagnosticStatus.OK,message="OK")
        #self.stat_bat_base.values = [ KeyValue("Voltage (V)",str(msg.voltage)),
        #                            KeyValue("Percentage",str(msg.percent)),
        #                            KeyValue("Charging",str(msg.plugged_in))]
        #
        #if msg.voltage < SystemInfo.BAT_VOLT_ERROR:
        #    self.stat_bat_base.level = DiagnosticStatus.ERROR
        #    self.stat_bat_base.message = "Battery almost empty"
        #elif msg.voltage < SystemInfo.BAT_VOLT_WARN:
        #    self.stat_bat_base.level = DiagnosticStatus.WARN
        #    self.stat_bat_base.message = "Battery almost empty"
        
        return msg

    def cb_base_bat(self, msg):
        self.base_bat_voltage = msg.voltage
        self.base_bat_watt = msg.watt
        self.base_bat_percent = msg.percent
        self.base_bat_temp = msg.temp
        self.base_bat_plugged_in = msg.plugged_in
    

        
if __name__ == '__main__':
    try:
        obj = SystemInfo()
    except rospy.ROSInterruptException:
        pass

