#!/usr/bin/env python  
#Please see the licensing information in license.txt
"""
References:

(1) The Biomimetics and Robotics Lab (BIRL) at Sun Yat-sen University has examples, demos, and tutorials.
    The idea came by studying their examples/demos particularly the demo using the kinect camera.
    Dr. Juan Rojas and his lab have created an excellent site to help others learn. 
    I initially used the publisher/subscriber concept that they used to learn, but then eliminated it and just
    directly calculated the joint angles and used the Baxter API to move the joints to the calculated
    positions. I did utilize how to get the joint angles using vectors and code snippets for getting a simple
    moving average filter.
    Here is the website address (i) and the address to the kinect demo (ii):
    (i)  https://github.com/birlrobotics/birl_baxter.wiki.git 
    (ii) https://github.com/birlrobotics/birlBaxter_demos/tree/master/kinect_based_arm_tracking


(2) The Baxter Research Robot SDK website has multiple examples/tutorials.
    Code snippets on how to have Baxter carry certain actions were used from this website.  
    http://sdk.rethinkrobotics.com/wiki/Main_Page
    http://api.rethinkrobotics.com/baxter_interface/html/baxter_interface-module.html

(3) ROS Tutorials on tf (transform) http://wiki.ros.org/tf/Tutorials

(4) Patrick Goebel has a two volume set called "ROS BY EXAMPLE".  This is an excellent reference and great way to
    learn. This program uses a concept from Vol 1, Chapter 10 on how to detect if the head frame has been detected.
    It is from his markers_from_tf.py script found in the skeleton_markers package located here:
    https://github.com/pirobot/skeleton_markers
    
  
(5) The NeuroScience and Robotics laboratory at Northwestern University had a Baxter example.
    Used vector operations and concepts about how to calculate joint angles.
    Website address is:  https://github.com/NxRLab/nxr_baxter

 
(6) "Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart.  Chapter 19 is devoted to making the robot talk using 
    pyttsx. Pyttsx is text to speech. Code was used from the book.  The code for the book is available at (i) and the documentation for
    pyttsx is available at (ii)
    (i)  https://github.com/osrf/rosbook
    (ii) https://pyttsx.readthedocs.io/en/latest/

(7) "Skeletal Joint Smoothing White Paper".  This is a very helpful paper that describes concepts behind the need for filtering kinect camera
    data.  It then describes 10 different filters.  Here is the web address for the paper.  
    https://msdn.microsoft.com/en-us/library/jj131429.aspx

(8) Skeleton_markers package by Patrick Goebel.  Here is the website:
    http://wiki.ros.org/skeleton_markers

"""

"""
ROS IMPORTS
"""
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

import tf

from geometry_msgs.msg import Point
import numpy as np
"""
PYTHON IMPORTS
"""
import math

"""
RETHINK IMPORTS
"""
import baxter_interface
from baxter_interface import CHECK_VERSION

"""
MY IMPORTS
"""
# placeholder
"""
HERE IS WHAT THIS CLASS DOES:
(1) It gets the skeleton information from the camera--- get_skeleton method
(2) It gets the user index which is important to know which user the openni_tracker
    program is tracking. The user index is checked every 2 seconds in the
    main program.  
(3) There are two filtering functions in this class
    a) Simple moving average filter
    b) Double moving average filter

    PLEASE LOOK AT THE MICROSOFT WHITE PAPER ON KINECT SMOOTHING FILTERS AT THE FOLLOWING ADDRESS:
    https://msdn.microsoft.com/en-us/library/jj131429.aspx

    There are 10 different filters described.  This package uses 3 of them.
    2 are in this class.  The third one is the adaptive double exponential
    smoothing filter in a separate class. 
(4) Once the angles are filtered, there is a function called
    check_angle_range that makes sure that the angle calculated
    is within Baxter's range of motion. 


"""
class Skeleton(object):
    def __init__(self):
        # The simple moving average uses the average of 10 values.  This is why filters is 10
        # Experiment with the number of values used to see if changing the value makes Baxter
        # more accurate.  For example, try 15, 20, or thirty values to average. Just realize
        # the more values used, the slower response or greater latency there may be. 
        self.filters = 20
        # The double moving average uses 5 values for the first average and then
        #    5 values for the second average which is why filters1 is 5
        self.filters1 = 10
        self.joint_angle = {'left':[[0.0]*self.filters for i in range(4)], 'right':[[0.0]*self.filters for i in range(4)]}
        self.joint_angles1 = {'left':[[0.0]*self.filters1 for i in range(4)], 'right':[[0.0]*self.filters1 for i in range(4)]}
        self.joint_angles_double = {'left':[[0.0]*self.filters1 for i in range(4)], 'right':[[0.0]*self.filters1 for i in range(4)]}
    
    def get_skeleton(self,user_index):
        skeleton = []
        left_shoulder = '/left_shoulder_'+user_index

        left_elbow = '/left_elbow_'+user_index

        left_hand = '/left_hand_'+user_index
    
        right_shoulder = '/right_shoulder_'+user_index

        right_elbow = '/right_elbow_'+user_index

        right_hand = '/right_hand_'+user_index
    
        head = '/head_'+user_index


        torso = '/torso_'+user_index

        base = '/openni_depth_frame'
        skeleton.append(left_shoulder)
        skeleton.append(left_elbow)
        skeleton.append(left_hand)
        skeleton.append(right_shoulder)
        skeleton.append(right_elbow)
        skeleton.append(right_hand)
        skeleton.append(head)
        skeleton.append(torso)
        skeleton.append(base)
        return skeleton

    def simple_moving_average_filter(self,limb_name, **angles):

        
        angle_s0 = angles["s0"]
        angle_s1 = angles["s1"]
        angle_e0 = angles["e0"]
        angle_e1 = angles["e1"]
    
        filters = self.filters
        

        joint_angles = self.joint_angle
       
        # popping off the last value from the dictionary of joints, then we will append the calculated angle value in radians
        # This popping off the first value in the dictionary and then appending to the last position is what
        #    makes this a moving average.
        for i in joint_angles[limb_name]:
            i.pop(0)
     
        
        joint_angles[limb_name][0].append(angles["s0"])
        joint_angles[limb_name][1].append(angles["s1"])
        joint_angles[limb_name][2].append(angles["e0"])
        joint_angles[limb_name][3].append(angles["e1"])

        #below is where we "smooth" data with a filter
        #the joint angle we just added has the heaviest weighted value.  
        # creating a list with the angles for s0,s1,e0,e1 respectively.
        filtered_angle = [0.0, 0.0, 0.0, 0.0]
        for i in range(0, 4):
            sum = 0
            for j in range(0, filters):
                sum = sum+joint_angles[limb_name][i][j]
            filtered_angle[i] = sum/filters

        angle_s0 = filtered_angle[0]
        angle_s1 = filtered_angle[1]
        angle_e0 = filtered_angle[2]
        angle_e1 = filtered_angle[3]
        return {'s0':angle_s0, 's1':angle_s1, 'e0':angle_e0, 'e1':angle_e1}
 
    def double_moving_average_filter(self,limb_name, **angles):
        
        
        angle_s0 = angles["s0"]
        angle_s1 = angles["s1"]
        angle_e0 = angles["e0"]
        angle_e1 = angles["e1"]
    
        filters1 = self.filters1
        
        joint_angles1 = self.joint_angles1
        
        joint_angles_double = self.joint_angles_double
        
    
        # popping off the last value from the dictionary of joints, then we will append the calculated angle value in radians
        # This popping off the first value in the dictionary and then appending to the last position is what
        #    makes this a moving average. 
        for i in joint_angles1[limb_name]:
            i.pop(0)
       
        for i in joint_angles_double[limb_name]:
            i.pop(0)
   
        joint_angles1[limb_name][0].append(angles["s0"])
        joint_angles1[limb_name][1].append(angles["s1"])
        joint_angles1[limb_name][2].append(angles["e0"])
        joint_angles1[limb_name][3].append(angles["e1"])
        
 
        # creating a list with the angles for s0,s1,e0,e1 respectively.
        filtered_angle = [0.0, 0.0, 0.0, 0.0]

        
        # Here we take the first moving average
        for i in range(0, 4):
            sum = 0
            for j in range(0, filters1):
                sum = sum+joint_angles1[limb_name][i][j]
            filtered_angle[i] = sum/filters1
        

        joint_angles_double[limb_name][0].append(filtered_angle[0])
        joint_angles_double[limb_name][1].append(filtered_angle[1])
        joint_angles_double[limb_name][2].append(filtered_angle[2])
        joint_angles_double[limb_name][3].append(filtered_angle[3])


        #Here we take the moving average of the average calculated above. 
        #This is what makes it a double moving average       
        for i in range(0, 4):
            sum = 0
            for j in range(0, filters1):
                sum = sum+joint_angles_double[limb_name][i][j]
            filtered_angle[i] = sum/filters1
        
        angle_s0 = filtered_angle[0]
        angle_s1 = filtered_angle[1]
        angle_e0 = filtered_angle[2]
        angle_e1 = filtered_angle[3]
        return {'s0':angle_s0, 's1':angle_s1, 'e0':angle_e0, 'e1':angle_e1}   
           

    def check_angle_range(self,limb_name, **filtered_angles):
        
        """
        VERY IMPORTANT!!!
        Before we published and set angles to the calculated position, we need a safety check.
        We are checking all the joint angles to make sure they are within Baxter's range.
        I substracted .25 to the max value and added .25 to the min value to create an additional buffer.
        My thought was that the 0.25 radian buffer accounts for possible joint calibration discrepancies.
        Published ranges are the following:
        s0:  -1.7 to 1.7
        s1:  -2.147 to 1.047
        e0:  -3.028 to -3.028
        e1:  -0.052 to 2.618
        w0:   N/A because it is set to 0.0
        w1:   N/A because it is set to 0.0
        w2:   N/A because it is set to 0.0
        """
    
        angle_s0 = filtered_angles["s0"]
        angle_s1 = filtered_angles["s1"]
        angle_e0 = filtered_angles["e0"]
        angle_e1 = filtered_angles["e1"]
        if angle_s0 > 1.45:
            angle_s0 = 1.45
            print("Modified s0 joint to 0.64")
        elif angle_s0 < -1.45:
            angle_s0 = -1.45
            print("Modified s0 joint to -1.45")
        else:
            angle_s0 = angle_s0

        if angle_s1 < -1.8:
            angle_s1 = -1.8
            print("Modified s1 joint to -1.8")
        elif angle_s1 > 0.8:
            angle_s1 = 0.8
            print('Modified s1 joint to 0.8')
        else:
            angle_s1 = angle_s1


        if angle_e0 < -2.77:
            angle_e0 = -2.77
            print('Modified e0 joint to -2.77')
        elif angle_e0 > 2.77:
            angle_e0 = 2.77
            print('Modified e0 joint to 2.77')
        else:
            angle_e0 = angle_e0

        if angle_e1 < 0.2:
            angle_e1 = 0.2
            print('Modified e1 joint to 0.2')
        elif angle_e1 > 2.36:
            angle_e1 = 2.36
            print('Modified e1 joint to 2.36')
        else:
            angle_e1 = angle_e1

        print "Final angles after smooth and safety check: ", limb_name, angle_s0, angle_s1, angle_e0, angle_e1

        return {'angle_s0':angle_s0, 'angle_s1':angle_s1, 'angle_e0':angle_e0, 'angle_e1':angle_e1}

    def get_user_index(self, listener):
        #figure out which user is detected
    
        skeleton_detected = False
        user_index = 99
        count = 0
        while not skeleton_detected:
            frames = [f for f in listener.getFrameStrings() if
                     f.startswith('head_')]
            print frames
            print len(frames)
            try:
                #if the head is visible, use it's index
                if count % 2 == 0 and len(frames)/2 <= 1:
                    head_frame = frames[1]
                elif count % 2 != 0 and len(frames)/2 <= 1:
                    head_frame = frames[0]
                elif count % 2 == 0 and len(frames) / 2 > 1:
                    index = len(frames)/2
                    head_frame = frames[index + 1]
                elif count % 2 != 0 and len(frames) / 2 > 1:
                    index = len(frames)/2
                    head_frame = frames[index]
                user_index = head_frame.replace('head_', '')
                print('This is the user index')
                print user_index
                #make sure we have a head frame before existing the loop
                # we need a user index
                try:
                    if user_index != 99 and len(user_index) <= 2:
                        skeleton_detected = True
                    else:
                        skeleton_detected = False
                        count += 1
                except:
                    skeleton_detected = False
            except:
                skeleton_detected = False
        return user_index

 
