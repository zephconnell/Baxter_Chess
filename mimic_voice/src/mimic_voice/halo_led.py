#!/usr/bin/env python

# Derived from code snippets from Rethink Robotics web site. 

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
import rospy
from std_msgs.msg import Float32
import baxter_interface
from baxter_interface import CHECK_VERSION

class Halo():
    def __init__(self):
       
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
    def red_color(self):
        print("Entering red_color method")
        
        red = 100.0
        green = 0.0
        self.red_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level',Float32, queue_size = 1)
        self.green_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32,queue_size = 1)
        rospy.sleep(1)
        self.red_pub.publish(red)
        rospy.sleep(1)
        self.green_pub.publish(green)
    def green_color(self):
        print("Entering green_color method")
        red = 0.0
        green = 100.0
        self.red_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_red_level',Float32, queue_size = 1)
        self.green_pub = rospy.Publisher('/robot/sonar/head_sonar/lights/set_green_level', Float32,queue_size = 1)
        rospy.sleep(1)
        self.red_pub.publish(red)
        rospy.sleep(1)
        self.green_pub.publish(green)
        
    def change_halo_color(self):
        print("Entering halo_color method")
        halo1 = Halo()
        for x in range (3):
            halo1.red_color()
            halo1.green_color()
        

def main():
    rospy.init_node("halo_color")
    rate = rospy.Rate(2)
    print("Starting main")
    halo= Halo()
    halo.change_halo_color()
    print("The halo lights were changed")


if __name__ =="__main__":
        main()
        
        
