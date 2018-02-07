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
import threading
import Queue
from std_msgs.msg import Float32
import baxter_interface
from baxter_interface import CHECK_VERSION
"""
ROS IMPORTS
"""
import rospy
import cv2
import cv_bridge
import rospkg

from sensor_msgs.msg import (
    Image
)

"""
Rethink IMPORTS
"""
#placeholder

from sensor_msgs.msg import JointState


from sensor_msgs.msg import JointState
import baxter_dataflow
import baxter_interface
from baxter_core_msgs.msg import EndEffectorState

from baxter_interface import CHECK_VERSION
"""
MY IMPORTS
"""
from pyttsx_client5 import TalkMimic
class HaloBlink(object):
    """
    Define Gripper class to use for demos. Using threading so that grippers
    simultaneously move.
    """
    def __init__(self):
        
        self._rs = baxter_interface.RobotEnable()

    def talk_mimic(self):
        talkmimic = TalkMimic()
        talkmimic.talk_client()

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
        
    
    def demo_halo_blink(self):
        """
        Runs halo_blink demo
        """
        try:
            
            def change_halo_color():
                print("Entering halo_color method")
                halo1 = HaloBlink()
                for x in range (3):
                    halo1.red_color()
                    halo1.green_color()
            def raise_eyebrows():
                print("Entering raise_eyebrows")
                _rp = rospkg.RosPack()
                _images = (_rp.get_path('mimic_voice') + '/share/images')
                for i in range(2):
                    
                    img = cv2.imread(_images + '/looking_left.jpg')
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
                    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
                    pub.publish(msg)
                    rospy.sleep(1.0)

                    img = cv2.imread(_images + '/looking_up.jpg')
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
                    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
                    pub.publish(msg)
                    rospy.sleep(1.0)

		    img = cv2.imread(_images + '/looking_right.jpg')
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
                    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
                    pub.publish(msg)
                    rospy.sleep(1.0)

                    img = cv2.imread(_images + '/looking_straight.jpg')
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
                    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
                    pub.publish(msg)
                    rospy.sleep(1.0)


                

            def demo_blink():
        

                def _blink(io):
               
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


            

            for i in range(1):
                t = threading.Thread(target = change_halo_color)
                t1 = threading.Thread(target = demo_blink)
                t2 = threading.Thread(target = raise_eyebrows)
                t.daemon = True
                t1.daemon = True
                t2.daemon = True
                t.start()
                t1.start()
                t2.start()
                baxter_dataflow.wait_for(
                    lambda: not (t.is_alive() or
                                 t1.is_alive()),
                    timeout = 30.0,
                    timeout_msg=("Timeout while waiting for gripper demo threads"
                                 " to finish"),
                    rate = 10,
                )
                t.join()
                t1.join()
                t2.join()
                rospy.sleep(1.0)

        except Exception:
            pass






