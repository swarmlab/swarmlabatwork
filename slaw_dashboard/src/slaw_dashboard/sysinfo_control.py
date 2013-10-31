import roslib
roslib.load_manifest('slaw_dashboard')

import wx

import math


from os import path

def non_zero(value):
  if value < 0.00001 and value > -0.00001:
    return 0.00001
  return value

class WifiControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(48, 48))
    self.SetBackgroundColour(parent.GetBackgroundColour())
    
    self._bitmaps = []
    self._bitmap_stall = wx.Bitmap(path.join(icons_path, "wifi-stall.png"), wx.BITMAP_TYPE_PNG)
    for i in range(5):
      self._bitmaps.append(wx.Bitmap(path.join(icons_path, "wifi-%d.png"%i), wx.BITMAP_TYPE_PNG))    

    self._stall = True
    self.SetToolTip(wx.ToolTip("Wifi: stall"))

    self._signallevel = -1

    self.Bind(wx.EVT_PAINT, self.on_paint)


  def set_stall(self):
    self._stall = True
    self._signallevel = -1
    self.SetToolTip(wx.ToolTip("Wifi: stall"))
    self.Refresh()

  def update(self, signallevel):
    self._stall = False
    self._signallevel = signallevel
    self.SetToolTip(wx.ToolTip("Wifi signal level: %.1f dBm"%signallevel))
    self.Refresh()


  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
    if self._stall or self._signallevel == -1:
      dc.DrawBitmap(self._bitmap_stall, 0, 0, True)
    else:
      level = - self._signallevel
      if level < 30:
        level = 30
      if level > 95:
        level = 95
      perc = 100 - (level-30) * 100.0/65.0
      idx = int( (perc + 12.5) / 25.0 )
      dc.DrawBitmap(self._bitmaps[idx], 0, 0, True)
      

class BatteryControl(wx.Window):
  def __init__(self, parent, id, icons_path, name):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(48, 48))
    self.SetBackgroundColour(parent.GetBackgroundColour())
    
    self._bitmaps = []
    self._bitmap_stall = wx.Bitmap(path.join(icons_path, "battery-stall.png"), wx.BITMAP_TYPE_PNG)
    self._bitmap_charging = wx.Bitmap(path.join(icons_path, "battery-charging.png"), wx.BITMAP_TYPE_PNG)
    for i in range(6):
      self._bitmaps.append(wx.Bitmap(path.join(icons_path, "battery-%d.png"%i), wx.BITMAP_TYPE_PNG))    

    self._stall = True        
    self._name = name
    self.SetToolTip(wx.ToolTip("%s: stall"%(self._name)))

    self._pct = -1
    self._voltage = -1
    self._time = -1
    self._charging = False
    self.Bind(wx.EVT_PAINT, self.on_paint)


  def set_stall(self):
    self._stall = True
    self._pct = -1
    self._voltage = -1
    self._watt = -1
    self._charging = False
    self.SetToolTip(wx.ToolTip("%s: stall"%(self._name)))
    self.Refresh()

  def update(self, pct, voltage, watt, charging):
    self._stall = False
    self._pct = pct
    self._voltage = voltage
    self._watt = watt
    self._charging = charging
    tooltip = "%s: %.0f%% (%.2f V, %.2f W)"%(self._name, pct, voltage, watt)
    self.SetToolTip(wx.ToolTip(tooltip))
    self.Refresh()


  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
    if self._stall or self._pct == -1 or self._voltage == -1:
      dc.DrawBitmap(self._bitmap_stall, 0, 0, True)
    else:
      idx = int( (self._pct + 10.0) / 20.0 ) 
      dc.DrawBitmap(self._bitmaps[idx], 0, 0, True)
      if self._charging:
        dc.DrawBitmap(self._bitmap_charging, 0, 0, True)


class NetworkControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(48, 48))
    self.SetBackgroundColour(parent.GetBackgroundColour())
    
    self._bitmap_stall = wx.Bitmap(path.join(icons_path, "network-stall.png"), wx.BITMAP_TYPE_PNG)
    self._bitmap_up = wx.Bitmap(path.join(icons_path, "network-up.png"), wx.BITMAP_TYPE_PNG)
    self._bitmap_down = wx.Bitmap(path.join(icons_path, "network-down.png"), wx.BITMAP_TYPE_PNG)

    self._stall = True
    self.SetToolTip(wx.ToolTip("Ethernet: stall"))

    self._connected = False
    self.Bind(wx.EVT_PAINT, self.on_paint)


  def set_stall(self):
    self._connected = False
    self.SetToolTip(wx.ToolTip("Ethernet: stall"))
    self.Refresh()

  def update(self, connected):
    self._stall = False
    self._connected = connected
    if self._connected:
      tooltip = "Ethernet: up"
    else:
      tooltip = "Ethernet: down"
    self.SetToolTip(wx.ToolTip(tooltip))
    self.Refresh()


  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
    if self._stall:
      dc.DrawBitmap(self._bitmap_stall, 0, 0, True)
    else:
      if self._connected:
        dc.DrawBitmap(self._bitmap_up, 0, 0, True)
      else:
        dc.DrawBitmap(self._bitmap_down, 0, 0, True)


class CPUControl(wx.Window):
  def __init__(self, parent, id, icons_path):
    wx.Window.__init__(self, parent, id, wx.DefaultPosition, wx.Size(48, 48))
    self.SetBackgroundColour(parent.GetBackgroundColour())
    
    self._bitmap_stall = wx.Bitmap(path.join(icons_path, "cpu-stall.png"), wx.BITMAP_TYPE_PNG)
    self._bitmap_bg = wx.Bitmap(path.join(icons_path, "cpu-bg.png"), wx.BITMAP_TYPE_PNG)

    self._stall = True
    self.SetToolTip(wx.ToolTip("CPU: stall"))

    self._cpu_usage_hist = []
    self._cpu_usage = -1
    self.Bind(wx.EVT_PAINT, self.on_paint)


  def set_stall(self):
    self._connected = False
    self.SetToolTip(wx.ToolTip("CPU: stall"))
    self.Refresh()

  def update(self, cpu_usage, cpu_temp):
    self._stall = False
    self._cpu_usage = cpu_usage
    self._cpu_usage_hist.insert(0, self._cpu_usage)
    if len(self._cpu_usage_hist) > 11:
      self._cpu_usage_hist.pop()
    tooltip = "CPU: %.0f%%, %.0fC"%(cpu_usage, cpu_temp)
    self.SetToolTip(wx.ToolTip(tooltip))
    self.Refresh()


  def on_paint(self, evt):
    dc = wx.BufferedPaintDC(self)
    dc.SetBackground(wx.Brush(self.GetBackgroundColour()))
    dc.Clear()
    if self._stall:
      dc.DrawBitmap(self._bitmap_stall, 0, 0, True)
    else:
      dc.DrawBitmap(self._bitmap_bg, 0, 0, True)
      
      for x in range(len(self._cpu_usage_hist)):
        pct = int(self._cpu_usage_hist[x] / 10.0)
        for y in range (pct):
          if y < 7:
            dc.SetPen(wx.Pen(wx.GREEN, 1, wx.SOLID))
          else:
            dc.SetPen(wx.Pen(wx.RED, 1, wx.SOLID))
          dc.DrawRectangle(38 - x*3, 41 - (y+1)*3, 2, 2)

class SysInfoControl(wx.StaticBoxSizer):
  def __init__(self, parent, id, title, icons_path):
    wx.StaticBoxSizer.__init__(self, wx.StaticBox(parent, id, title), wx.HORIZONTAL)

    # Battery Base
    self._battery_base_ctrl = BatteryControl(parent, wx.ID_ANY, icons_path, "Base")
    self.Add(self._battery_base_ctrl, 1, wx.EXPAND)
    
    # Battery PC
    self._battery_pc_ctrl = BatteryControl(parent, wx.ID_ANY, icons_path, "PC")
    self.Add(self._battery_pc_ctrl, 1, wx.EXPAND)

    # Wifi
    self._wifi_ctrl = WifiControl(parent, wx.ID_ANY, icons_path)
    self.Add(self._wifi_ctrl, 1, wx.EXPAND)

    # Ethernet
    self._net_ctrl = NetworkControl(parent, wx.ID_ANY, icons_path)
    self.Add(self._net_ctrl, 1, wx.EXPAND)

    # CPU
    self._cpu_ctrl = CPUControl(parent, wx.ID_ANY, icons_path)
    self.Add(self._cpu_ctrl, 1, wx.EXPAND)




  def update(self, msg):
    self._battery_base_ctrl.update(msg.battery_base.percent, msg.battery_base.voltage, msg.battery_base.watt, msg.battery_base.plugged_in)
    self._battery_pc_ctrl.update(msg.battery_pc.percent, msg.battery_pc.voltage, msg.battery_pc.watt, msg.battery_pc.plugged_in)
    self._wifi_ctrl.update(msg.network.wifi_signallevel)
    self._net_ctrl.update(msg.network.ethernet_connected)
    self._cpu_ctrl.update(msg.system.cpu_usage_average, msg.system.cpu_temp_average)

  def set_stall(self):
    self._battery_base_ctrl.set_stall()
    self._battery_pc_ctrl.set_stall()
    self._wifi_ctrl.set_stall()
    self._net_ctrl.set_stall()
    self._cpu_ctrl.set_stall()



