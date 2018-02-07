#!/usr/bin/env python 
#Please see license.txt in the package folder
""" 
REFERENCES

NeuroScience and Robotics laboratory at Northwestern University:
https://github.com/NxRLab/nxr_baxter/blob/debug_kinect_restart/src/vector_operations.py 
written by Jon Rivera

Biomimetics and Robotics Lab (BIRL)
https://github.com/birlrobotics/birlBaxter_demos/blob/master/kinect_based_arm_tracking/scripts/tf_listen

These are the vector operations for the mimic program.
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


class Vector():

    def __init__(self):
        pass

    def angle_between_vectors(self,A,B):
        """
        A dot B = |A| * |B| * cos(angle)
        """
        AdotB = np.dot(A,B)
        a = np.linalg.norm(A) #this gives the magnitude of the vector A
        b = np.linalg.norm(B) # this gives the magnitude of the vector B
        angle = np.arccos(AdotB/(a*b))
        #print 'Below is the angle', angle
    
        return angle
    

    def vector_cross_product(self,a, b):
        r1 = a[1]*b[2]-b[1]*a[2]
        r2 = a[2]*b[0]-b[2]*a[0]
        r3 = a[0]*b[1]-b[0]*a[1]
        return (r1, r2, r3)

    def make_vector_from_coordinates(self,p1,p2):
        return [ p2.x - p1.x, p2.y - p1.y, p2.z - p1.z]
    


        
    
