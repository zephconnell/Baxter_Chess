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
import rospy

import cv2
import cv_bridge
import rospkg
from display_camera import Camera

from sensor_msgs.msg import (
    Image
)

from pyttsx_client6 import Request
from pyttsx_client7 import Thanks
from pyttsx_client8 import ThankYou_Letter
from std_msgs.msg import Header

import baxter_interface
from baxter_interface import CHECK_VERSION


class RequestPaper:
    def __init__(self):
        self._rp = rospkg.RosPack()
        self._images = (self._rp.get_path('mimic_voice') + '/share/images')
        self._rs = baxter_interface.RobotEnable()
        self._rs.enable()
        self._left = baxter_interface.Limb('left')
        

    def nod(self):
        head = baxter_interface.Head()
        for _ in xrange(3):
            head.command_nod()

    def reset_picture(self):
        print("Entered reset_picture function")
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1)

    def camera_message(self):
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1)

    def request_paper(self):
        print ("entering request_paper method")
        r = RequestPaper()
        left = baxter_interface.Limb('left')
        print("requesting paper")
        gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        gripper.calibrate()
        img = cv2.imread(self._images + '/looking_left.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(0.5)
        request = Request()
        request.request_client()
        zero_angle = self._left.joint_angles()
        zero_angle['left_s0'] = 0.0
        zero_angle['left_s1'] = 0.0
        zero_angle['left_e0'] = 0.0
        zero_angle['left_e1'] = 0.0
        zero_angle['left_w0'] = 0.0
        zero_angle['left_w1'] = 0.0
        zero_angle['left_w2'] = 0.0
        left.move_to_joint_positions(zero_angle)
        gripper.open()
        rospy.sleep(3.0)
        gripper.close()
        r.nod()
        rospy.sleep(0.1)
        #say thank you.
        thanks = Thanks()
        thanks.say_thanks()
        #img = cv2.imread(self._images + '/smileyFace2.jpg')
        img = cv2.imread(self._images + '/looking_straight.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub.publish(msg)
        rospy.sleep(0.5)
        #camera = Camera()
        #camera.close_left_hand_camera()
        #camera.open_head_camera()
        #camera.display()
        

        position1 = {'left_e0': 0.081, 'left_e1': 0.027, 'left_s0': -0.042, 'left_s1': -0.039, 'left_w0': -1.539, 'left_w1': -0.0947, 'left_w2': -0.0168}
        position2 = {'left_e0': -1.188, 'left_e1': 1.32, 'left_s0': -0.1886, 'left_s1': -0.1710, 'left_w0': -0.9782, 'left_w1': 1.4818, 'left_w2': 0.490}
        position3 = {'left_e0': -1.071, 'left_e1': 1.77, 'left_s0': -0.395, 'left_s1': -0.572, 'left_w0': -2.09, 'left_w1': 2.077, 'left_w2': 1.012}
        position4 = {'left_e0': -1.071, 'left_e1': 1.77, 'left_s0': -0.395, 'left_s1': -0.572, 'left_w0': -2.09, 'left_w1': 2.077, 'left_w2': 3.03}
        letter_by_face = {'left_e0': -1.533, 'left_e1': 1.33, 'left_s0': -0.439, 'left_s1': -0.047, 'left_w0': -1.469, 'left_w1': 1.565, 'left_w2': 2.65}
        letter_closer = {'left_e0': -1.45, 'left_e1': 1.9, 'left_s0': -0.402, 'left_s1': -0.117, 'left_w0': -1.66, 'left_w1': 1.65, 'left_w2': -0.0552}
        letter_closer1 = {'left_e0': -1.37, 'left_e1': 1.98, 'left_s0': -0.352, 'left_s1': -0.09, 'left_w0': -1.582, 'left_w1': 1.77, 'left_w2': -0.04}
        show_crowd1 = {'left_e0': -1.21, 'left_e1': 0.3064, 'left_s0': -0.6619, 'left_s1': -0.056, 'left_w0': -1.877, 'left_w1': 1.6524, 'left_w2': 0.292}
        show_crowd2 = {'left_e0': -1.21, 'left_e1': 0.3064, 'left_s0': -0.6619, 'left_s1': -0.056, 'left_w0': -1.877, 'left_w1': 1.6524, 'left_w2': 1.156}
        show_crowd3 = {'left_e0': -1.21, 'left_e1': 0.3064, 'left_s0': -0.6619, 'left_s1': -0.056, 'left_w0': -1.877, 'left_w1': 1.6524, 'left_w2': -0.692}
        for x in range(1):
            left.move_to_joint_positions(position1)
            left.move_to_joint_positions(position2)
            left.move_to_joint_positions(letter_by_face)
            left.move_to_joint_positions(letter_closer)
            left.move_to_joint_positions(letter_closer1)
            rospy.sleep(0.5)
            left.move_to_joint_positions(zero_angle)
        img = cv2.imread(self._images + '/looking_left.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(0.5)
        thanks = ThankYou_Letter()
        thanks.thank_you_letter()
        gripper.open()
        #say thank you
        img = cv2.imread(self._images + '/looking_straight.jpg')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub.publish(msg)
        rospy.sleep(0.5)
        print('exit hello method')
        rospy.sleep(1.0)
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
    rospy.init_node("look_at_paper")
    req = RequestPaper()
    
    req.request_paper()

if __name__ =="__main__":
        main()

