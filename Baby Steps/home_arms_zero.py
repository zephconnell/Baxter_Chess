#! /usr/bin/env python 

#Script to move baxter's arms to zero position
# Adapted from "ROS Robotics By Example" by Carol Fairchild and Dr. Thomas L. Harman

"""
in order for the program to work, you need to import the ROS python interface
which is called rospy.  You also need Baxter's interface because it contains
python classes with built in methods that you will use to control Baxter.  This 
interface is called baxter_interface

Checking the software version is a good practice to verify the software version.
It makes sure that our software running on the robot is compatible with the
version of the software development kit we are using. 
Currently, we are using version 1.2
"""
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION


# ROS requires that we initialize a node.  Below is the code to do that. 
rospy.init_node('Home_Arms')

"""this line creates an instance of the RobotEnable class which is a sub-module
   of the baxter_interface. Here is where we check our version of software.
   The class RobotEnable has methods that allow us to check check the robot's
        last known state (.state).  Here, we created a variable called
         init_state that took our instance of the baxter_interface (rs), found
         the state of the robot and then set it to whatever the current
         enabled state which could be True or False, depending on if the
         robot's motors are already enabled.  This variable
         will be used as a reference point later in the program when we
         do a clean-shutdown on baxter. 
"""

rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled

# here is the method that when we enter ctrl-c will put Baxter
# back into neutral position and disable his motors. 
def clean_shutdown():
    # this line just prints information to the screen
    print("/nExiting example.")
    #the baxter_interface has built in methods that move his
    # arms to neutral
    limb_right.move_to_neutral()
    limb_left.move_to_neutral()
    if not init_state:
        print("Disabling robot...")
        rs.disable()
#This code tell ROS that when we enter ctrl-c, it should start the clean_shutdown method
rospy.on_shutdown(clean_shutdown)
#This enables Baxter's motors
rs.enable()
#The next two lines create instances of Baxter's Class limb. 
# This class has many methods that we will use.
# You have to create an instance for each arm. 
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')


#Create two variable that describe the joint positions that you want each arm to assume
home_right_zero = {'right_s0': 0.0, 'right_s1': 0.0, 'right_e0': 0.0, 'right_e1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0} 
home_left_zero = {'left_s0': 0.0, 'left_s1': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0}
 
# use the Limb class method -- move_to_joint_position to have Baxter's arms move
# to where you want them to go
limb_right.move_to_joint_positions(home_right_zero)
limb_left.move_to_joint_positions(home_left_zero)

#simply keeps python from exiting until the node is stopped with ctrl-c
rospy.spin()
