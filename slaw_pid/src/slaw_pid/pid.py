# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, UC Regents
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of California nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('slaw_pid')
import rospy

class PidController(object):
    """
    Very simple PID controller class. Inspired by, and similar interface to,
    pid.cpp from pr2_controllers/control_toolbox
    """
    def __init__(self, KP=0, KI=0, KD=0, Ilimit=0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        assert Ilimit >= 0
        self.Imax = Ilimit
        self.Imin = -Ilimit
        self.zero()
        
    def zero(self):
        self.i_term = 0.0
        self.p_term = 0.0
        self.d_term = 0.0
        self.p_error_last = 0.0
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0
        self.current_output = 0.0
        
    def update(self, error, dt=0.0, error_dot=None):
        self.p_error = error
        if error_dot is None:
            # TODO: finite differencing
            if dt > 0.0:
                self.d_error = (self.p_error - self.p_error_last) / dt
                self.p_error_last = self.p_error
        else:
            self.d_error = error_dot
            
        if dt == 0:
            output = 0.0
        else:
            p_term = self.KP*self.p_error
            self.i_error = self.i_error + dt*self.p_error
            i_term = self.KI*self.i_error
            
            if i_term > self.Imax:
                i_term = self.Imax
                self.i_error = i_term/self.KI
            elif i_term < self.Imin:
                i_term = self.Imin
                self.i_error = i_term/self.KI
                
            d_term = self.KD*self.d_error
            output = -p_term - i_term - d_term
            self.p_term, self.i_term, self.d_term = p_term, i_term, d_term # maybe useful for debugging
            self.current_output = output
        return output
    
    def get_current_cmd(self):
        return self.current_output
