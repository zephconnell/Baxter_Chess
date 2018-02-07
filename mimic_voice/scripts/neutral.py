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
Reference is the Baxter SDK examples
"""
import rospy
from std_msgs.msg import String

import baxter_interface

from baxter_interface import CHECK_VERSION

import mimic_voice

from sensor_msgs.msg import Image

"""
WHAT THE RESET CLASS DOES
1) Puts Baxter's arm back to neutral position
2) Resets his face screen to the Rethink Logo Picture
3) Disables the motors
"""

class Reset(object):

    def __init__(self):
        
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self._pi = mimic_voice.PickImage()
        
        
       
    def set_neutral(self):
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()
       

    def reset(self):
      
        print('Resetting picture')
        print("Picture reset")
        self._pi.reset_picture()

    
       

    def clean_shutdown(self):
        print("\nRobot almost back to all neutral positions...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        else:
            print("Need to disable robot...")
            self._rs.disable()
        return True

                       
def main():       
                          
    
    print("Initializing node...")
    rospy.init_node("Reset_Baxter", anonymous = True)
    reset = Reset()
    reset.set_neutral()
    reset.reset()
    reset.clean_shutdown()
    
   
    print("Baxter's arms,led display are now back to neutral positions and he has been disabled.")

if __name__ == '__main__':
    main()       


