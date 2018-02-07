#!/usr/bin/env python 
# please see the license in license.txt
""" 
References are below:
 How to calculate the joints was taken from two sources
 (1)  a demo found at the Biomimetics and Robtics Lab (BIRL) website. The website link is
  below.  It has tutorials and demos to use to help learn about robotics and ROS
  https://github.com/birlrobotics/birlBaxter_demos/tree/master/kinect_based_arm_tracking
 
 (2) NeuroScience and Robotics laboratory at Northwestern University:
     https://github.com/NxRLab/nxr_baxter/blob/debug_kinect_restart/src/mimic.py

Of note, there is also a paper that describes another alternative way of calculating
the joints.  Here is the information below

Reddivari, C. Yang, Z. Ju, P. Liang, Z. Li and B. Xu, Teleoperation Control of Baxter Robot
using Body Motion Tracking, presented at the 2014 IEEE International Conference on Multisensor Fusion
and Information Integration, Beijing, China, September 28-30, 2014


This class calculates the joint angles from the transform of user coordinates to
Baxter coordinates.
Once the angles are calculated, we still need to filter them and check that
the angles fall within Baxter's range of motion.  This is done in another
class.
"""

"""
ROS IMPORTS
"""
import rospy

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
from vector import Vector


class CalculateJoints(object):

    def __init__(self):
        pass
        

    """
    Joint s0 is different for the right arm versus the left arm.
    if it is the right arm, it will move towards the back as the value gets more
    negative and towards the chest as it gets move positive.

    if it is the left arm, it will move towards the back the greater or more positive the value and the
      left arm moves in toward the torso, the less or more negative the angle becomes. 

    Neutral position is an s0 joint angle of 0.0.  This does not put the arm perpendicular with the torso,
    but more at pi/4.  

    So you have to use different calculations for both the right and left joint at s0 because they move in 
    opposite directions. 
    
    Neutral is defined as:

    ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
    [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]

    Published joint ranges from Baxter's URDF are the following:
    Baxter's URDF is found here:
    https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_description/urdf/baxter_base/baxter_base.urdf.xacro
    s0:  -1.7 to 1.7
    s1:  -2.147 to 1.047
    e0:  -3.028 to -3.028
    e1:  -0.052 to 2.618
    w0:   N/A because it is set to 0.0
    w1:   N/A because it is set to 0.0
    w2:   N/A because it is set to 0.0


    """
        
    def calculate_s0(self, vector2, s_2_s_x, limb_name):
        vector = Vector()
        theta1 = vector.angle_between_vectors(vector2, s_2_s_x)
        #left s0 moves backward the greater or more positive the number
        if limb_name == 'left': 
            joint_s0 = theta1 - math.pi/4

        #right s0 moves forward (toward the chest) the greater or more positive the number        
        else:
            joint_s0 = theta1 - 3*math.pi/4
                   
        return joint_s0

    def calculate_s1(self, head_2_t, s_2_e):
        vector = Vector()
        #get the angle between vector1(head_2_t) and shoulder_2_elbow
        theta_2 = vector.angle_between_vectors(head_2_t, s_2_e)
        
        """
        For the s1 joint, it moves in the same direction for both the right and left limb.
        At neutral position, it sits at roughly pi/2 from the torso. 

        If the angle between the line from the head to torso and shoulder to elbow is small, then 
        s1 will have a greater value.  As you can see below, as the value of s1 increases or becomes more positive,
        the shoulder will move down.
 
        If the value of s1 increases, the shoulder will move down
        If the value of s1 decreases, the shoulder will move up
        """
        joint_s1 = math.pi/2-theta_2
        print 'radius_s1: ', joint_s1
        return joint_s1

    def calculate_e0(self,s_2_e, e_2_hand, vector1, limb_name):
        vector = Vector()
        vector2 = vector.vector_cross_product(s_2_e, e_2_hand)
    
   
        theta_3 = vector.angle_between_vectors(vector1,vector2)
    
        """
        The e0 joint, just like the s0 joint moves in opposite directions for the right and left
        The e0 joint controls how much Baxter's elbow twists from side to side.  In neutral, it is at 0.0
        As the left e0 value increases, the arm/elbow twists inward and as it decreases, it abducts from the body.
        As the right e0 value increases, the arm/elbow twists outward (abducting) from the body and vice versa.

        Here, since this joint moves(twists or rolls) perpendicular to the s0 joint, we find the angle between two vectors
        that are cross-products between s_2_e (shoulder to elbow) and head_2_t (head to torso) i.e. vector1 and vector 2 which is the cross-product between
        the s_2_e and e_2_hand.  
        """
        joint_e0 = theta_3
        if limb_name == 'left':
            joint_e0 = -joint_e0
        return joint_e0

    def calculate_e1(self, s_2_e, e_2_hand):
        """
        This joint calculation is straight forward, we just
        use the joint above and below the joint i.e. the shoulder and hand.
        To find e0, we use the line from the shoulder to elbow, and from the elbow
        to hand and then calculate the angle inbetween them.
        """
        vector = Vector()
        theta_4 = vector.angle_between_vectors(s_2_e,e_2_hand)
        print 'theta_4: ', theta_4
        joint_e1 = theta_4
        return joint_e1

  

        
