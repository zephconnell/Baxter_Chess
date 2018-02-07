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

import collections
from copy import deepcopy
import rospy

import cv2
import cv_bridge
import rospkg

from sensor_msgs.msg import (
    Image
)


from std_msgs.msg import Header

import baxter_interface

from baxter_interface import CameraController
from pyttsx_client import Talk
from pyttsx_client1 import TalkLeft
from pyttsx_client2 import TalkRight
from pyttsx_client3 import TalkLights
from pyttsx_client5 import TalkMimic

class PickImage:
    def __init__(self):
        self._rp = rospkg.RosPack()
        self._images = (self._rp.get_path('mimic_voice') + '/share/images')
        self._path = self._rp.get_path('mimic_voice') + '/cfg/'
        self._rs = baxter_interface.RobotEnable()
        self._rs.enable()

    def nod(self):
        head = baxter_interface.Head()
        for _ in xrange(2):
            head.command_nod()

    def reset_picture(self):
        print("Entered reset_picture function")
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)

    def camera_message(self):
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1)

    def hello(self):
        head = baxter_interface.Head()
        print("entered hello method")
        for _ in xrange(4):
            head.command_nod()
        head.set_pan(1.0, 0.5)
        head.set_pan(-1.0,0.5)
        head.set_pan(0.0,0.5)
        img = cv2.imread(self._images + '/hello_baxter1.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)
        talk = Talk()
        talk.talk_client()
        
        #img = cv2.imread(self._images + '/smileyFace2.jpg')
        img = cv2.imread(self._images + '/looking_straight.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub.publish(msg)
        rospy.sleep(1.0)
        print('exit hello method')

    def hello_basic(self):
        head = baxter_interface.Head()
        print("entered hello method")
        for _ in xrange(4):
            head.command_nod()
        head.set_pan(1.0, 0.5)
        head.set_pan(-1.0,0.5)
        head.set_pan(0.0,0.5)
        img = cv2.imread(self._images + '/hello_baxter1.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)
        talk = Talk()
        talk.talk_client()
        talk1 = TalkMimic()
        talk1.talk_client()
        
        #img = cv2.imread(self._images + '/smileyFace2.jpg')
        img = cv2.imread(self._images + '/looking_straight.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub.publish(msg)
        rospy.sleep(1.0)
        print('exit hello method')

    def look_left(self):
        talkl = TalkLeft()
        talkl.talk_client()
        img = cv2.imread(self._images + '/looking_left.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)

    def look_right(self):
        talkr = TalkRight()
        talkr.talk_client()
        img = cv2.imread(self._images + '/looking_right.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)

    def look_straight(self):
        img = cv2.imread(self._images + '/looking_straight.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1.0)

    def talk_lights(self):
        talkli = TalkLights()
        talkli.talk_client()
        
def main():
    rospy.init_node("show_images")
    pi = PickImage()
    
    pi.hello()

if __name__ =="__main__":
        main()
