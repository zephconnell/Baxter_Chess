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

"""
ROS IMPORTS
"""
import rospy

"""
Rethink IMPORTS
"""
"""
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
)
"""
from sensor_msgs.msg import JointState


from sensor_msgs.msg import JointState
import baxter_dataflow
import baxter_interface
from baxter_core_msgs.msg import EndEffectorState

from baxter_interface import CHECK_VERSION
"""
MY IMPORTS
"""
from pyttsx_client4 import TalkGrippers

class Gripper(object):
    """
    Define Gripper class to use for demos. Using threading so that grippers
    simultaneously move.
    """
    def __init__(self):
        self._rs = baxter_interface.RobotEnable()

    def demo_gripper(self):
        """
        Runs gripper demo
        """
        grip_talk = TalkGrippers()
        grip_talk.talk_client()
        try:
            #print "Enabling robot, Moving to Neutral Location..."
            self._rs.enable()
            right = baxter_interface.Limb('right')
            

            left = baxter_interface.Limb('left')
            

            def gripper():
                print "Move right arm to neutral position"
                right.move_to_neutral()
                rospy.sleep(1.0)
                name = 'right'
                gripper = baxter_interface.Gripper(name, CHECK_VERSION)
                s_topic = 'robot/end_effector/' + name + '_gripper/state'
                ee_state = rospy.wait_for_message(s_topic,
                                                  EndEffectorState,
                                                  5.0
                                                  )
                print "Calibrating %s Gripper" % (name.capitalize(),)
                gripper.calibrate()
                print "Close %s Gripper" % (name.capitalize(),)
                gripper.close(True)
                print "Open %s Gripper" % (name.capitalize(),)
                gripper.open(True)
                print "Showing %s Gripper Velocity Moves" % (name.capitalize(),)
                gripper.set_moving_force(100.0)
                gripper.set_velocity(50.0)
                gripper.close(True)
                gripper.set_velocity(25.0)
                gripper.open(True)
                gripper.set_velocity(100.0)
                gripper.close(True)
                gripper.open(True)
                gripper.set_velocity(50.0)
                gripper.set_moving_force(30.0)
                gripper.close(True)
                gripper.open(True)
                print "Finished demo gripper on right"

            def gripper1():
                print "Move left arm to neutral position"
                left.move_to_neutral()
                rospy.sleep(1.0)
                name = 'left'
                gripper = baxter_interface.Gripper(name, CHECK_VERSION)
                s_topic = 'robot/end_effector/' + name + '_gripper/state'
                ee_state = rospy.wait_for_message(s_topic,
                                                  EndEffectorState,
                                                  5.0
                                                  )
                
                print "Calibrating %s Gripper" % (name.capitalize(),)
                gripper.calibrate()
                print "Close %s Gripper" % (name.capitalize(),)
                gripper.close(True)
                print "Open %s Gripper" % (name.capitalize(),)
                gripper.open(True)
                print "Showing %s Gripper Velocity Moves" % (name.capitalize(),)
                gripper.set_moving_force(100.0)
                gripper.set_velocity(50.0)
                gripper.close(True)
                gripper.set_velocity(25.0)
                gripper.open(True)
                gripper.set_velocity(100.0)
                gripper.close(True)
                gripper.open(True)
                gripper.set_velocity(50.0)
                gripper.set_moving_force(30.0)
                gripper.close(True)
                gripper.open(True) 
                print "Finished demo of gripper on left"

            for i in range(1):
                t = threading.Thread(target = gripper)
                t1 = threading.Thread(target = gripper1)
                t.daemon = True
                t1.daemon = True
                t.start()
                t1.start()
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
                rospy.sleep(1.0)

        except Exception:
            pass










