#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
References:
The Rethink Robotics website has example code. 
Many code snippets were taken from these examples,
baxter_tools, specifically a python file called smoketests.py
https://github.com/RethinkRobotics/baxter_tools/blob/master/src/baxter_tools/smoketests.py
"""
"""
PYTHON IMPORTS
"""

import rospy
import rospkg

"""
Rethink Imports
"""
from sensor_msgs.msg import JointState

import baxter_dataflow
import baxter_interface

from baxter_core_msgs.msg import AnalogIOStates
    

from baxter_interface import CHECK_VERSION


class BlinkLD(object):
    """
    Demos Baxter's LED lights on his arms
    """
    def __init__(self):
        self._rs = baxter_interface.RobotEnable()
        
    def demo_blink(self):
        

        def _blink(io):
            """
            Toggle itb lights.
            """
            for i in range(9):
                io.set_output(i % 2)
                rospy.sleep(0.1)

        try:
            itb_names = (
                'left_outer_light',
                'left_inner_light',
                'right_outer_light',
                'right_inner_light',
                'left_outer_light',
                'left_inner_light',
                'right_outer_light',
                'right_inner_light',
                )

            for itb in itb_names:
                print "Test: Blink %s" % itb
                io = baxter_interface.DigitalIO(itb)
                _blink(io)
            
        except Exception:
            pass
    def main():
        rospy.init_node("show lights")
        light = BlinkLD()
        light.demo_blink()

if __name__ =="__main__":
        main()


