#!/usr/bin/env python

import rospy
import std_srvs.srv

from baxter_core_msgs.srv import(
  ListCameras,
)
import sys
import baxter_interface

from baxter_interface.camera import CameraController

import subprocess

rospy.init_node("camera", anonymous = True)

"""
It is very important to have the rospy.sleep
inbetween each call to the camera service.
You must allow a minimum of 10 seconds which
is why the sleep was set to 11 seconds.
If not, then a subsequent call to the camera 
service is coming from the same node and
you will get an error. 
"""

listCameras = 'rosrun baxter_tools camera_control.py -l'
closeLeft = 'rosrun baxter_tools camera_control.py -c left_hand_camera'
closeHead = 'rosrun baxter_tools camera_control.py -c head_camera'
openLeft = 'rosrun baxter_tools camera_control.py -o left_hand_camera -r 960x600'

subprocess.Popen(listCameras, shell=True)
rospy.sleep(11)
subprocess.Popen(closeLeft,shell = True)
rospy.sleep(11)
print("The left hand camera was closed")
subprocess.Popen(closeHead, shell = True)
rospy.sleep(11)
print("The head camera was closed")
subprocess.Popen(openLeft, shell = True)
print("The left hand camera was opened at resolution 960 by 600")

