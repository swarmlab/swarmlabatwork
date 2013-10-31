# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# based on turtlebot_dashboard and youbot_dashboard

import roslib
roslib.load_manifest('slaw_dashboard')

import math

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import std_msgs.msg
from slaw_diagnostics.msg import SysInfo, BatteryStatus
from std_srvs.srv import Empty

import actionlib
from move_base_msgs.msg import MoveBaseAction

import rospy
from rosgraph import rosenv

from os import path
import threading
import subprocess

from status_control import StatusControl
from rosout_frame import RosoutFrame
from sysinfo_control import *
from slaw_location_service.srv import *

from pr2_msgs.msg import PowerBoardState
from diagnostics_frame import DiagnosticsFrame

class Dashboard(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='SLAW Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        self.SetBackgroundColour(wx.Colour(242,241,240,255))
        
        self.SetTitle('SLAW dashboard (%s)'%rosenv.get_master_uri())

        rospy.init_node('slaw_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("slaw_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
               
        slaw_icons_path = path.join(roslib.packages.get_pkg_dir('slaw_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)


        # Diagnostics Panel
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagnostics"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, slaw_icons_path, "diagnostics", False)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, slaw_icons_path, "rosout", False)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)
        
        # SmachViewer
        self._smach_button = StatusControl(self, wx.ID_ANY, slaw_icons_path, "smach", False)
        self._smach_button.SetToolTip(wx.ToolTip("Smach Viewer"))
        static_sizer.Add(self._smach_button, 0)
        self._smach_button.Bind(wx.EVT_LEFT_DOWN, self.on_smach_button_clicked)
        self._smach_button.set_ok()
        
        # Platform
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Platform"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # base status
        self._base_status = StatusControl(self, wx.ID_ANY, slaw_icons_path, "wheel", False)
        self._base_status.SetToolTip(wx.ToolTip("Base Motors: Stale"))
        static_sizer.Add(self._base_status, 0)
        self._base_status.Bind(wx.EVT_LEFT_DOWN, self.on_base_status_clicked)
        
        # arm status
        self._arm_status = StatusControl(self, wx.ID_ANY, slaw_icons_path, "arm", False)
        self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Stale"))
        static_sizer.Add(self._arm_status, 0)
        self._arm_status.Bind(wx.EVT_LEFT_DOWN, self.on_arm_status_clicked)
        
        
        # Driver
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Drv"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # driver status
        self._driver_status = StatusControl(self, wx.ID_ANY, slaw_icons_path, "motor", False)
        self._driver_status.SetToolTip(wx.ToolTip("Driver: Stale"))
        static_sizer.Add(self._driver_status, 0)
        self._driver_status.Bind(wx.EVT_BUTTON, self.on_driver_status_clicked)

                                       
        # System info
        self._robot_sysinfo = SysInfoControl(self, wx.ID_ANY, "System", slaw_icons_path)
        sizer.Add(self._robot_sysinfo)
      

        # Apps                        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Apps"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

        # Clear costmap
        self._costmap_ctrl = StatusControl(self, wx.ID_ANY, slaw_icons_path, "costmap", False)
        self._costmap_ctrl.SetToolTip(wx.ToolTip("Clear costmap"))
        static_sizer.Add(self._costmap_ctrl, 0)
        self._costmap_ctrl.Bind(wx.EVT_BUTTON, self.on_costmap_clicked)

        # Cancel goals
        self._goal_ctrl = StatusControl(self, wx.ID_ANY, slaw_icons_path, "cancel-goal", False)
        self._goal_ctrl.SetToolTip(wx.ToolTip("Cancel goal"))
        static_sizer.Add(self._goal_ctrl, 0)
        self._goal_ctrl.Bind(wx.EVT_BUTTON, self.on_goal_clicked)
        
        # Tuck arm
        self._arm_ctrl = StatusControl(self, wx.ID_ANY, slaw_icons_path, "arm-tuck", False)
        self._arm_ctrl.SetToolTip(wx.ToolTip("Tuck arm"))
        static_sizer.Add(self._arm_ctrl, 0)
        self._arm_ctrl.Bind(wx.EVT_BUTTON, self.on_arm_clicked)
        
        # Store location
        self._location_ctrl = StatusControl(self, wx.ID_ANY, slaw_icons_path, "nav", False)
        self._location_ctrl.SetToolTip(wx.ToolTip("Store location"))
        static_sizer.Add(self._location_ctrl, 0)
        self._location_ctrl.Bind(wx.EVT_BUTTON, self.on_location_clicked)
        
        # Wrap up stuff
        self._config = wx.Config("slaw_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)

        # Message subscribers
        self._last_sysinfo_message = 0.0
        self._last_power_state_message = 0.0
        self._last_platform_state_message = 0.0
        
        self._platform_state_message = None
 
        self._sub_sysinfo = rospy.Subscriber('dashboard/sysinfo', SysInfo, self.cb_sysinfo)
        self._sub_platform_state = rospy.Subscriber('dashboard/platform_state', PowerBoardState, self.cb_platform_state)
        
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', 1)
            self._srv_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            self._costmap_ctrl.set_ok()
            self._costmap_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self._costmap_ctrl.set_stale()
            self._costmap_ok = False            
        
        self._goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if self._goal_client.wait_for_server(rospy.Duration(1)):
            self._goal_ctrl.set_ok()
            self._goal_ok = True
        else:
            rospy.logwarn("move base not running yet, can't cancel goal")
            self._goal_ctrl.set_stale()
            self._goal_ok = False    
             
        try:
            rospy.wait_for_service('/tuck_arm', 1)
            self._srv_arm = rospy.ServiceProxy('/tuck_arm', Empty)
            self._arm_ctrl.set_ok()
            self._arm_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("/tuck_arm service unavailable")
            self._arm_ctrl.set_stale()
            self._arm_ok = False
        
        self._srv_store_location = None
        self._srv_delete_location = None
        self._srv_get_location = None
        try:
            rospy.wait_for_service('location_service/store_location', 1)
            self._srv_store_location = rospy.ServiceProxy('location_service/store_location', StoreLocation)
            self._srv_delete_location = rospy.ServiceProxy('location_service/delete_location', DeleteLocation)
            self._srv_get_location = rospy.ServiceProxy('location_service/get_location', GetLocation)
            self._location_ctrl.set_ok()
            self._location_ok = True
        except rospy.ROSException, e:
            rospy.logwarn("location service unavailable")
            self._location_ctrl.set_stale()
            self._location_ok = False


    def __del__(self):
        self._sub_sysinfo.unregister()
        self._sub_power_state.unregister()
        self._sub_platform_state.unregister()
        self._srv_costmap.unregister()
        self._srv_arm.unregister()
        self._srv_store_location.unregister()
        self._srv_delete_location.unregister()
        self._goal_client.unregister()

        
    def on_timer(self, evt):
        level = self._diagnostics_frame.get_top_level_state()
        if (level == -1 or level == 3):
            if (self._diagnostics_button.set_stale()):
                self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
        elif (level >= 2):
            if (self._diagnostics_button.set_error()):
                self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
        elif (level == 1):
            if (self._diagnostics_button.set_warn()):
                self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
        else:
            if (self._diagnostics_button.set_ok()):
                self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))
        
        self.update_rosout()
        
        if (rospy.get_time() - self._last_sysinfo_message > 5.0):
            self._robot_sysinfo.set_stall()
               
        if (rospy.get_time() - self._last_platform_state_message > 5.0):
            self._arm_status.set_stale()
            self._base_status.set_stale()
            self._driver_status.set_stale()
                    
            ctrls = [self._arm_status, self._base_status, self._driver_status]
            for ctrl in ctrls:
                ctrl.SetToolTip(wx.ToolTip("No platform_state message received in the last 5 seconds"))
        else:
            self._driver_status.set_ok()
            self._driver_status.SetToolTip(wx.ToolTip("Restart driver"))
        
        if (rospy.is_shutdown()):
            self.Close()

    def on_diagnostics_clicked(self, evt):
        self._diagnostics_frame.Show()
        self._diagnostics_frame.Raise()
        
    def on_rosout_clicked(self, evt):
        self._rosout_frame.Show()
        self._rosout_frame.Raise()
        
        
    def on_smach_button_clicked(self, evt):
        subprocess.call(["rosrun", "smach_viewer", "smach_viewer.py"])

    
    def on_costmap_clicked(self, evt):
        if not self._costmap_ok: 
            self._srv_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        try:            
            self._srv_costmap()
            self._costmap_ctrl.set_ok()
            self._costmap_ok = True
            rospy.loginfo("Costmap cleared")
        except rospy.ServiceException, e:
            rospy.logwarn("move base not running yet, can't clear costmap")
            self._costmap_ctrl.set_stale()
            self._costmap_ok = False


    def on_goal_clicked(self, evt):
        if not self._goal_ok:
            if self._goal_client.wait_for_server(rospy.Duration(1)):
                self._goal_ctrl.set_ok()
                self._goal_ok = True
            else:
                rospy.logwarn("move base not running yet, can't cancel goal")
                self._goal_ctrl.set_stale()
                self._goal_ok = False
                return

        self._goal_client.cancel_all_goals()
        rospy.loginfo("All goals cancelled")
        
        
    def on_arm_clicked(self, evt):
        if not self._arm_ok: 
            self._srv_arm = rospy.ServiceProxy('/tuck_arm', Empty)    
        
        try:
            self._srv_arm()
            self._arm_ctrl.set_ok()
            self._arm_ok = True
            rospy.loginfo("Arm tucked")
        except rospy.ServiceException, e:
            rospy.logwarn("/tuck_arm service unavailable")
            self._arm_ctrl.set_stale()
            self._arm_ok = False
    
    
    def on_location_clicked(self, evt):
        menu = wx.Menu()
        menu.Bind(wx.EVT_MENU, self.on_store_location, menu.Append(wx.ID_ANY, "Save current location"))
        menu.Bind(wx.EVT_MENU, self.on_delete_location, menu.Append(wx.ID_ANY, "Delete location"))
        menu.Bind(wx.EVT_MENU, self.on_get_location, menu.Append(wx.ID_ANY, "Show locations"))
        self.PopupMenu(menu)
    
    
    def on_store_location(self, evt):
        name = ""
        input_box = wx.TextEntryDialog(self,"Name:","","")
        if input_box.ShowModal()==wx.ID_OK:
            name=input_box.GetValue()
        else:
            return
        
        if self._srv_store_location is None:
            self._srv_store_location = rospy.ServiceProxy('location_service/store_location', StoreLocation)    
        
        try:
            result = self._srv_store_location(name)
            self._location_ctrl.set_ok()
            self._location_ok = True
            if result.success:
                rospy.loginfo("Location %s stored", name)
            else:
                rospy.logwarn("Storing location %s failed: %s", name, result.reason)
        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self._location_ctrl.set_stale()
            self._location_ok = False
    
    
    def on_delete_location(self, evt):
        name = ""
        input_box = wx.TextEntryDialog(self,"Name:","","(empty to delete all)")
        if input_box.ShowModal()==wx.ID_OK:
            name=input_box.GetValue()
        else:
            return
        
        if self._srv_delete_location is None:
            self._srv_delete_location = rospy.ServiceProxy('location_service/delete_location', DeleteLocation)
        
        try:
            result = self._srv_delete_location(name)
            self._location_ctrl.set_ok()
            self._location_ok = True
            if result.success:
                rospy.loginfo(result.reason)
            else:
                rospy.logwarn("Deleting location %s failed: %s", name, result.reason)
        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self._location_ctrl.set_stale()
            self._location_ok = False
    
    
    def on_get_location(self, evt):
        if self._srv_get_location is None:
            self._srv_get_location = rospy.ServiceProxy('location_service/get_location', GetLocation)
        
        try:
            result = self._srv_get_location("")
            self._location_ctrl.set_ok()
            self._location_ok = True
            
            message = "Name:\t\t(x, y)\n=============\n\n"
            for location in result.locations:
                message = message + "%s:\t\t\t(%.2f, %.2f)\n"%(location.name, location.pose.position.x, location.pose.position.y)
            
            wx.MessageBox(message, 'Locations', wx.OK | wx.ICON_INFORMATION)
            
        except rospy.ServiceException, e:
            rospy.logwarn("location service unavailable")
            self._location_ctrl.set_stale()
            self._location_ok = False
            
        
    def on_base_status_clicked(self, evt):
        menu = wx.Menu()
        menu.Bind(wx.EVT_MENU, self.on_base_switch_on, menu.Append(wx.ID_ANY, "Enable Base Motors"))
        menu.Bind(wx.EVT_MENU, self.on_base_switch_off, menu.Append(wx.ID_ANY, "Disable Base Motors"))
        self._base_status.toggle(True)
        self.PopupMenu(menu)
        self._base_status.toggle(False)
      
    def on_base_switch_on(self, evt):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_STANDBY):
                switch_on_base = rospy.ServiceProxy("/base/switchOnMotors", Empty)
            
                try:
                    switch_on_base()
                except rospy.ServiceException, e:
                    wx.MessageBox("Failed to switch ON base motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
            
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_ENABLED):
                wx.MessageBox("Base motors are already switched ON", "Error", wx.OK|wx.ICON_ERROR)
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_DISABLED):
                wx.MessageBox("Base is not connected", "Error", wx.OK|wx.ICON_ERROR)
              
    def on_base_switch_off(self, evt):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_ENABLED):
                switch_off_base = rospy.ServiceProxy("/base/switchOffMotors", Empty)

                try:
                    switch_off_base()
                except rospy.ServiceException, e:
                    wx.MessageBox("Failed to switch OFF base motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_STANDBY):
                wx.MessageBox("Base motors are already switched OFF", "Error", wx.OK|wx.ICON_ERROR)
            elif (self._platform_state_message.circuit_state[0] == PowerBoardState.STATE_DISABLED):
                wx.MessageBox("Base is not connected", "Error", wx.OK|wx.ICON_ERROR)
      
    def on_arm_status_clicked(self, evt):
        menu = wx.Menu()
        menu.Bind(wx.EVT_MENU, self.on_arm_switch_on, menu.Append(wx.ID_ANY, "Enable Arm Motors"))
        menu.Bind(wx.EVT_MENU, self.on_arm_switch_off, menu.Append(wx.ID_ANY, "Disable Arm Motors"))
        self._arm_status.toggle(True)
        self.PopupMenu(menu)
        self._arm_status.toggle(False)
      
    def on_arm_switch_on(self, evt):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_STANDBY):
                switch_on_arm = rospy.ServiceProxy("/arm_1/switchOnMotors", Empty)
       
                try:
                    switch_on_arm()
                except rospy.ServiceException, e:
                    wx.MessageBox("Failed to switch ON arm motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_ENABLED):
                wx.MessageBox("Arm motors are already switched ON", "Error", wx.OK|wx.ICON_ERROR)
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_DISABLED):
                wx.MessageBox("Arm is not connected", "Error", wx.OK|wx.ICON_ERROR)
              
    def on_arm_switch_off(self, evt):
        # if any of the breakers is not enabled ask if they'd like to enable them
        if (self._platform_state_message is not None):
            if (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_ENABLED):
                switch_off_arm = rospy.ServiceProxy("/arm_1/switchOffMotors", Empty)
       
                try:
                    switch_off_arm()
                except rospy.ServiceException, e:
                    wx.MessageBox("Failed to switch OFF arm motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
          
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_STANDBY):
                wx.MessageBox("Arm motors are already switched OFF", "Error", wx.OK|wx.ICON_ERROR)
            elif (self._platform_state_message.circuit_state[1] == PowerBoardState.STATE_DISABLED):
                wx.MessageBox("Arm is not connected", "Error", wx.OK|wx.ICON_ERROR)

    def on_driver_status_clicked(self, evt):
        if (self._platform_state_message is not None):        
            reconnect = rospy.ServiceProxy("/reconnect", Empty)

            try:
                reconnect()
            except rospy.ServiceException, e:
                wx.MessageBox("Failed to reconnect the driver: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)


    def cb_sysinfo(self, msg):
        wx.CallAfter(self.update_sysinfo, msg)

    def update_sysinfo(self, msg):
        self._last_sysinfo_message = rospy.get_time()
        self._robot_sysinfo.update(msg)


    def cb_platform_state(self, msg):
        wx.CallAfter(self.update_platform_state, msg)
      
    def update_platform_state(self, msg):
        self._platform_state_message = msg
        self._last_platform_state_message = rospy.get_time()


        if (msg.circuit_state[0] == PowerBoardState.STATE_ENABLED):
            self._base_status.set_ok()
            self._base_status.SetToolTip(wx.ToolTip("Base Motors: Switched ON"))
        elif (msg.circuit_state[0] == PowerBoardState.STATE_STANDBY):
            self._base_status.set_warn()
            self._base_status.SetToolTip(wx.ToolTip("Base Motors: Switched OFF"))
        elif (msg.circuit_state[0] == PowerBoardState.STATE_DISABLED):
            self._base_status.set_error()
            self._base_status.SetToolTip(wx.ToolTip("Base Motors: not connected"))
        else:
            self._base_status.set_stale()
            self._base_status.SetToolTip(wx.ToolTip("Base Motors: stale"))

        if (msg.circuit_state[1] == PowerBoardState.STATE_ENABLED):
            self._arm_status.set_ok()
            self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Switched ON"))
        elif (msg.circuit_state[1] == PowerBoardState.STATE_STANDBY):
            self._arm_status.set_warn()
            self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: Switched OFF"))
        elif (msg.circuit_state[1] == PowerBoardState.STATE_DISABLED):
            self._arm_status.set_error()
            self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: not connected"))
        else:
            self._arm_status.set_stale()
            self._arm_status.SetToolTip(wx.ToolTip("Arm Motors: stale"))

          
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      self.Destroy()
      
