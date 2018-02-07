#!/usr/bin/env python  
# Please see license.txt in the package folder.  All the different open
#    source licenses that the references use are listed in the license.txt
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
from sensor_msgs.msg import Image
import tf

from geometry_msgs.msg import Point
import numpy as np
"""
PYTHON IMPORTS
"""
import threading
import Queue
from copy import copy

"""
RETHINK IMPORTS
"""
import baxter_interface
from baxter_interface import CHECK_VERSION
import baxter_dataflow

"""
MY IMPORTS
"""
import mimic_voice
from mimic_voice import Skeleton
from mimic_voice import CalculateJoints
from mimic_voice import ADESfilter
from mimic_voice import Vector
from mimic_voice import Gripper
from mimic_voice import BlinkLD
from mimic_voice import HaloBlink
from mimic_voice import PickImage
from pyttsx_client import Talk
from pyttsx_client1 import TalkLeft
from pyttsx_client2 import TalkRight
from pyttsx_client3 import TalkLights
from mimic_voice import request
from mimic_voice import RequestPaper

"""
_________________________________________________________________
WHAT THE MIMIC CLASS DOES


It uses the ROS transform library to obtain skeleton coordinates
  via the XBOX 360 kinect camera and "transform" them to 
  Baxter's frame. 

The class relies on outside classes to get the user index, get the 
skeleton coordinates, vector functions, calculate the joint angles,
filter the calculated joint angles, and also do a safety check to make
sure that the calculated joint angles fall within Baxter's range
of motion.  There is also a timer to check the user index to make
sure that the user has not changed. If the user index has changed,
then the program will call a function to get the updated user.  

____________________________________________________________________
WHAT MIMIC_VOICE_DEMOB DOES

DemoB uses pyttsx to make Baxter talk
1) Baxter says hello and welcomes everyone
2) Baxter asks to look at a letter, extends his left arm to
   take it, looks at it, then gives it back
3) Then Baxter starts the mimic part of the program
_____________________________________________________________________

"""

class MimicVoiceB():
    
    def __init__(self):

        self.skeleton = mimic_voice.Skeleton()
        self.a = mimic_voice.ADESfilter()
        self.vector = mimic_voice.Vector()
        self.rate = rospy.Rate(50) 
        self.limb_1 = "left"
        self.limb_2 = "right"
        self.listener = tf.TransformListener()
        self.joint_list = {'left_s0':0.0,'left_s1':0.0,'left_e0':0.0,
                    'left_e1':0.0, 'left_w0': 0.0,'left_w1': 0.0, 'left_w2': 0.0,'right_s0':0.0,'right_s1':0.0,
                    'right_e0':0.0,'right_e1':0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}

        self.prev_joint_list = {'left_s0':0.0,'left_s1':0.0,'left_e0':0.0,
                    'left_e1':0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0,'right_s0':0.0,'right_s1':0.0,
                    'right_e0':0.0,'right_e1':0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()
        self.left_limb = baxter_interface.limb.Limb("left")
        self.right_limb = baxter_interface.limb.Limb("right")
        self.right_w0 = 0.0
        self.right_w1 = 0.0
        self.right_w2 = 0.0
        self.left_w0 = 0.0
        self.left_w1 = 0.0
        self.left_w2 = 0.0
        self.command = 'test'
        self.prev_w0 = 0.0

        
        rospy.Subscriber('right_w0', Float64, self.right_w0_callback)
        rospy.Subscriber('/right_w1', Float64, self.right_w1_callback)
        rospy.Subscriber('/right_w2', Float64, self.right_w2_callback)

        rospy.Subscriber('left_w0', Float64, self.left_w0_callback)
        rospy.Subscriber('left_w1', Float64, self.left_w1_callback)
        rospy.Subscriber('left_w2', Float64, self.left_w2_callback)
        rospy.Subscriber('/command', String, self.command_callback)
        

    def get_right_w0(self):
        return self.right_w0

    def get_right_w1(self):
        return self.right_w1

    def get_right_w2(self):
        return self.right_w2

    def get_left_w0(self):
        return self.left_w0

    def get_left_w1(self):
        return self.left_w1

    def get_left_w2(self):
        return self.left_w2

    def get_command(self):
        return self.command

    def right_w0_callback(self, msg):
        self.right_w0 = msg.data

    def right_w1_callback(self, msg):
        self.right_w1 = msg.data

    def right_w2_callback(self, msg):
        self.right_w2 = msg.data

    def left_w0_callback(self, msg):
        self.left_w0 = msg.data

    def left_w1_callback(self, msg):
        self.left_w1 = msg.data

    def left_w2_callback(self, msg):
        self.left_w2 = msg.data

    def command_callback(self, msg):
        self.command = msg.data

    def move_to_position(self,pi):
        #Move Baxter's arms to a position where the user will be able to see his LED lights blink. 
        wave_r = {'right_s0':0.0, 'right_s1': -0.55, 'right_e0':0.0, 'right_e1': 0.75, 'right_w0':-0.836, 'right_w1': 1.26,
              'right_w2':0.0}

        wave_l = {'left_s0':0.0, 'left_s1': -0.55, 'left_e0':0.0, 'left_e1': 0.75, 'left_w0':0.836, 'left_w1': 1.26,
              'left_w2':0.0}
        for i in range(1):
                t = threading.Thread(target = pi.look_left())
                t1 = threading.Thread(target = self.left_limb.move_to_joint_positions(wave_l))
                t.daemon = True
                t1.daemon = True
                t.start()
                t1.start()
                baxter_dataflow.wait_for(
                    lambda: not (t.is_alive() or
                                 t1.is_alive()),
                    timeout = 20.0,
                    timeout_msg=("Timeout while waiting for demo threads"
                                 " to finish"),
                    rate = 10,
                )
                t.join()
                t1.join()
                rospy.sleep(1.0)

        for i in range(1):
                t = threading.Thread(target = pi.look_right())
                t1 = threading.Thread(target = self.right_limb.move_to_joint_positions(wave_r))
                t.daemon = True
                t1.daemon = True
                t.start()
                t1.start()
                baxter_dataflow.wait_for(
                    lambda: not (t.is_alive() or
                                 t1.is_alive()),
                    timeout = 20.0,
                    timeout_msg=("Timeout while waiting for demo threads"
                                 " to finish"),
                    rate = 10,
                )
                t.join()
                t1.join()
                rospy.sleep(1.0)

        rospy.sleep(1)
        pi.look_straight()

    def move_joints(self,limb_name, **joint_list):
        """
        Baxter's 3 wrist joints are set to zero because we cannot
        directly transform the human skeleton with 3 arm joints to 
        Baxter's seven arm joints.

        The program continues until you enter Ctrl C for shutdown or
        You can pause by using Ctrl Z.  The pause allows you
        to change the user so many people can try the mimic.
        To restart type fg then press enter
        """
        
        joint_left_s0 = joint_list["left_s0"]
        joint_left_s1 = joint_list["left_s1"]
        joint_left_e0 = joint_list["left_e0"]
        joint_left_e1 = joint_list["left_e1"]
        joint_left_w0 = joint_list["left_w0"]
        joint_left_w1 = joint_list["left_w1"]
        joint_left_w2 = joint_list["left_w2"]

        joint_right_s0 = joint_list["right_s0"]
        joint_right_s1 = joint_list["right_s1"]
        joint_right_e0 = joint_list["right_e0"]
        joint_right_e1 = joint_list["right_e1"]
        joint_right_w0 = joint_list["right_w0"]
        joint_right_w1 = joint_list["right_w1"]
        joint_right_w2 = joint_list["right_w2"]
        
        angles1 =self.left_limb.joint_angles()
        angles1 = {'left_s0': 0.0, 'left_s1': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_w0': 0.0,
                       'left_w1': 0.0, 'left_w2': 0.0}
        
        angles1['left_s0'] = joint_left_s0
        angles1['left_s1'] = joint_left_s1
        angles1['left_e0'] = joint_left_e0
        angles1['left_e1'] = joint_left_e1
        angles1['left_w0'] = joint_left_w0
        angles1['left_w1'] = joint_left_w1
        angles1['left_w2'] = joint_left_w2
        """  
        Note that you can use either 1) move_to_joint_positions or 2) set_joint_positions methods.

        1)set_joint_positions commands the joints of Baxter's limb to the specified joint angles.
        2)move_to_joint_positions waits until the reported joint state matches the joint angle specified.  It has a timeout parameter 
         and smooths the movement via filter. 
        Comment or uncomment whichever way you wish to move Baxter's arms to the angle. 
        """
        self.left_limb.set_joint_positions(angles1)
        #self.left_limb.move_to_joint_positions(angles1,threshold = 0.5)

        angles1 = self.right_limb.joint_angles()
        angles1 = {'right_s0': 0.0, 'right_s1': 0.0, 'right_e0': 0.0, 'right_e1': 0.0,
                      'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0}

        angles1['right_s0'] = joint_right_s0
        angles1['right_s1'] = joint_right_s1
        angles1['right_e0'] = joint_right_e0
        angles1['right_e1'] = joint_right_e1
        angles1['right_w0'] = joint_right_w0
        angles1['right_w1'] = joint_right_w1
        angles1['right_w2'] = joint_right_w2
        
        self.right_limb.set_joint_positions(angles1)
        #self.right_limb.move_to_joint_positions(angles1,threshold = 0.5)
            
        rate.sleep()
        
    def transform_to_angles(self, shoulder_sameSide, shoulder_oppositeSide, elbow, hand,  torso, head, base, listener, limb_name):
        
        try:
            """
            We want to transform from the base frame which is the camera frame (openni_depth_frame)
             to the 'skeleton joint frame' at the present time.  This is why we use rospy.Time(0)
            This returns us a set of coordinates x,y,z
            Here is the ROS tutorial web page on transform listener 
             http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
            """
            s1 = Point()
            # b_2_s_same is base to same side shoulder
            (b_2_s_same, rot) = listener.lookupTransform(base, shoulder_sameSide, rospy.Time(0))
            s1.x = b_2_s_same[0]
            s1.y = b_2_s_same[1]
            s1.z = b_2_s_same[2]
           
            s2 = Point()
            # transfrom from base to opposite shoulder
            (b_2_s_opposite, rot) = listener.lookupTransform(base, shoulder_oppositeSide, rospy.Time(0))
            s2.x = b_2_s_opposite[0]
            s2.y = b_2_s_opposite[1]
            s2.z = b_2_s_opposite[2]

            e = Point()
            # transform from base to elbow
            (b_2_e, rot) = listener.lookupTransform(base, elbow, rospy.Time(0))
            e.x = b_2_e[0]
            e.y = b_2_e[1]
            e.z = b_2_e[2]

            h = Point()
            #transform from base to hand
            (b_2_hand, rot) = listener.lookupTransform(base, hand, rospy.Time(0))
            h.x = b_2_hand[0]
            h.y = b_2_hand[1]
            h.z = b_2_hand[2]

            #transform from base to torso
            t = Point()
            (b_2_t, rot) = listener.lookupTransform(base, torso, rospy.Time(0))
            t.x = b_2_t[0]
            t.y = b_2_t[1]
            t.z = b_2_t[2]

            #transform from base to head
            hd = Point()
            (b_2_head, rot) = listener.lookupTransform(base, head, rospy.Time(0))
            hd.x = b_2_head[0]
            hd.y = b_2_head[1]
            hd.z = b_2_head[2]
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return {}

        #START MAKING THE VECTORS NEEDED TO FIND THE RELEVANT ANGLES
        
        #This is the vector between the two shoulders
        s_same_2_s_opposite = v.make_vector_from_coordinates(s1,s2)

        #This is the vector between the same side shoulder and the torso
        s_same_2_t = v.make_vector_from_coordinates(s1,t)
        

        #This is the vector between the same side shoulder and the elbow
        s_same_2_e = v.make_vector_from_coordinates(s1,e)
     
        #This is the vector between the elbow and the hand
        e_2_hand = v.make_vector_from_coordinates(e,h)

        #This is the vector between the head and the torso
        head_2_t = v.make_vector_from_coordinates (hd,t)
    
      
        # vector1 is the orthogonal vector to the vectors created
        # from the same side shoulder to elbow and the head to torso
        vector1 = v.vector_cross_product(s_same_2_e, head_2_t)

        """
        START CALCULATING THE JOINT ANGLES
            Instantiate an object of the class CalculateJoints
              so that we can call its methods
        """
        joint = mimic_voice.CalculateJoints()

        #find the s0 joint
        joint_s0 = joint.calculate_s0(vector1, s_same_2_s_opposite, limb_name)
    
    
        #find the s1 joint
        joint_s1 = joint.calculate_s1(head_2_t,s_same_2_e)

        #get the e0 joint value
        joint_e0 = joint.calculate_e0(s_same_2_e, e_2_hand, vector1, limb_name)
    
        #calculate e1 joint
        joint_e1 = joint.calculate_e1(s_same_2_e, e_2_hand)
    
       
        return {"s0":joint_s0, "s1":joint_s1, "e0":joint_e0, "e1":joint_e1}
    
    def person_to_baxter(self, shoulder_sameSide, shoulder_oppositeSide, elbow, hand, torso, head, base, listener, limb_name, prev_joint_list, command, **joint_list):
        #print('\n' *4)
        #print ("The value of prev_joint_list is: ", prev_joint_list)
        #rint ("The value of joint_list is: ", joint_list)
        #print('\n' *4)

        print('\n' * 2)
        print("Here is the value of command: ", command)
        print('\n' * 2)
        if command == 'stop':
            
            joint_list = prev_joint_list
            return (joint_list, prev_joint_list)
        
        #call transform_to_angles
        # we are sending in the skeleton coordinates from the openni_depth_frame
        first_angles = mime.transform_to_angles(shoulder_sameSide, shoulder_oppositeSide, elbow, hand, torso, head, base, listener, limb_name)
        
        if first_angles == {}:
            print "The coordinate transform failed and had an exception."
            return
    
    
        """
        Choose which filter you wish to use by commenting or uncommenting
           the choices below. 
           These filters are described in the microsoft white paper on skeleton
           filtering found at the following website:
           https://msdn.microsoft.com/en-us/library/jj131429.aspx
         ________________________________________________________________
        Choice 1 ---simple_moving_average_filter
        Choice 2 ---double_moving_average_filter
        Choice 3 ---ades_filter which stands for adaptive double exponential 
                       smoothing filter
                    
        The default is the double moving average but please try the others to see
         what works best.  I will also be working on the Jitter Removal Filter to
         see if this helps further smooth out Baxter's movements. His arms still have
         slight movement even when the user's arms are still. The Jitter filter would be
         used in combination with another filter.          
        """
        # Choice 1
        filtered_angles = mime.skeleton.simple_moving_average_filter(limb_name, **first_angles)

        # Choice 2
        #filtered_angles = mime.skeleton.double_moving_average_filter(limb_name, **first_angles)

        # Choice 3
        #filtered_angles = mime.a.ades_filter(limb_name,a, **first_angles)

    
        #Now we do a safety check to make sure the calculated angles are in Baxter's range
        # Joint angles will be modified if they are found to be out of range
        checked_angles = mime.skeleton.check_angle_range(limb_name,**filtered_angles)
        #print (" The value of checked_angles is: ", checked_angles)

        joint_list[limb_name+'_s0'] = checked_angles['angle_s0']
        joint_list[limb_name+'_s1'] = checked_angles['angle_s1']
        joint_list[limb_name+'_e0'] = checked_angles['angle_e0']
        joint_list[limb_name+'_e1'] = checked_angles['angle_e1']

        

        if limb_name == 'left':
    
            joint_list[limb_name+'_w0'] = mime.get_left_w0()
            joint_list[limb_name+'_w1'] = mime.get_left_w1()
            joint_list[limb_name+'_w2'] = mime.get_left_w2()

            if abs(joint_list['left_s0'] - prev_joint_list['left_s0']) < 0.1:
                joint_list['left_s0'] = prev_joint_list['left_s0']
                print("Used prev joint list for left_s0")

            if abs(joint_list['left_s0'] - prev_joint_list['left_s0']) >= 0.1:
                joint_list['left_s0'] = joint_list['left_s0']
                prev_joint_list['left_s0'] = joint_list['left_s0']

            if abs(joint_list['left_s1'] - prev_joint_list['left_s1']) < 0.1:
                joint_list['left_s1'] = prev_joint_list['left_s1']
                print("Used prev joint list for left_s1")

            if abs(joint_list['left_s1'] - prev_joint_list['left_s1']) >= 0.1:
                joint_list['left_s1'] = joint_list['left_s1']
                prev_joint_list['left_s1'] = joint_list['left_s1']

            if abs(joint_list['left_e0'] - prev_joint_list['left_e0']) < 0.1:
                joint_list['left_e0'] = prev_joint_list['left_e0']
                print("Used prev joint list for left_e0")

            if abs(joint_list['left_e0'] - prev_joint_list['left_e0']) >= 0.1:
                joint_list['left_e0'] = joint_list['left_e0']
                prev_joint_list['left_e0'] = joint_list['left_e0']

            if abs(joint_list['left_e1'] - prev_joint_list['left_e1']) < 0.1:
                joint_list['left_e1'] = prev_joint_list['left_e1']
                print("Used prev joint list for left_e1")

            if abs(joint_list['left_e1'] - prev_joint_list['left_e1']) >= 0.1:
                joint_list['left_e1'] = joint_list['left_e1']
                prev_joint_list['left_e1'] = joint_list['left_e1']
                
           
        else:

            joint_list[limb_name+'_w0'] = mime.get_right_w0()
            joint_list[limb_name+'_w1'] = mime.get_right_w1()
            joint_list[limb_name+'_w2'] = mime.get_right_w2()

            if abs(joint_list['right_s0'] - prev_joint_list['right_s0']) <= 0.1:
                joint_list['right_s0'] = prev_joint_list['right_s0']
                print("Used prev joint list for right_s0")

            if abs(joint_list['right_s0'] - prev_joint_list['right_s0']) > 0.1:
                joint_list['right_s0'] = joint_list['right_s0']
                prev_joint_list['right_s0'] = joint_list['right_s0']

            if abs(joint_list['right_s1'] - prev_joint_list['right_s1']) <= 0.1:
                joint_list['right_s1'] = prev_joint_list['right_s1']
                print("Used prev joint list for right_s1")

            if abs(joint_list['right_s1'] - prev_joint_list['right_s1']) > 0.1:
                joint_list['right_s1'] = joint_list['right_s1']
                prev_joint_list['right_s1'] = joint_list['right_s1']

            if abs(joint_list['right_e0'] - prev_joint_list['right_e0']) <= 0.1:
                joint_list['right_e0'] = prev_joint_list['right_e0']
                print("Used prev joint list for right_e0")

            if abs(joint_list['right_e0'] - prev_joint_list['right_e0']) > 0.1:
                joint_list['right_e0'] = joint_list['right_e0']
                prev_joint_list['right_e0'] = joint_list['right_e0']

            if abs(joint_list['right_e1'] - prev_joint_list['right_e1']) <= 0.1:
                joint_list['right_e1'] = prev_joint_list['right_e1']
                print("Used prev joint list for right_e1")

            if abs(joint_list['right_e1'] - prev_joint_list['right_e1']) > 0.1:
                joint_list['right_e1'] = joint_list['right_e1']
                prev_joint_list['right_e1'] = joint_list['right_e1']

           
        prev_joint_list = copy(joint_list)
        return (joint_list, prev_joint_list) 
        
    def my_callback(self,event):
        print("my_callback was called")
        return mime.skeleton.get_user_index(listener)   

    def set_neutral(self):
        
        self.left_limb.move_to_neutral()
        self.right_limb.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self.set_neutral()
        rospy.sleep(1)
        pi = mimic_voice.PickImage()
        pi.reset_picture()
        
        print("Disabling robot...")
        self._rs.disable()
        return True
    
if __name__ == '__main__':
    try:
        rospy.init_node('mimic_voice_demoB', anonymous=True)
        mime = MimicVoiceB()
        rospy.on_shutdown(mime.clean_shutdown)
        a = mime.a
        v = mime.vector
        rate = mime.rate
        limb_1 = mime.limb_1
        limb_2 = mime.limb_2
        listener = mime.listener
        joint_list = mime.joint_list
        right_w1 = mime.right_w1
        right_w2 = mime.right_w2
        left_w1 = mime.left_w1
        left_w2 = mime.left_w2
        prev_joint_list = mime.prev_joint_list
        command = mime.command

        # start the demo
        #Instantiate an object of the class PickImage to demo the hello method
        #talk_now.talk_client()
        pi = mimic_voice.PickImage()
        pi.hello()
        rospy.sleep(1)

        #Instantiate an object of the class Request so that Baxter can take a
        # piece of paper
        print("Going to ask for paper")
        requestPaper = mimic_voice.RequestPaper()
        requestPaper.request_paper()
        

        #Instantiate an object of the HaloBlink class
        lights = mimic_voice.HaloBlink()
        lights.talk_mimic()

        #figure out which user is detected by calling the user_index function
        user_index = mime.skeleton.get_user_index(listener)
        user_index_check = rospy.Timer(rospy.Duration(2), mime.my_callback)
        coordinates = mime.skeleton.get_skeleton(user_index)

        while not rospy.is_shutdown():
            # if the user_index changed i.e. we changed users, we need to get the user index again
            if user_index_check != user_index:
                if not rospy.is_shutdown():
                    print("We need to get the user_index again")
                    print('\n' * 10)
                    user_index = mime.skeleton.get_user_index(listener)
                    user_index_check = user_index

                    # Since the user index changed, we need to get the skeleton frames again   
                    coordinates = mime.skeleton.get_skeleton(user_index)
                
                    rospy.sleep(1)
      
            command = mime.get_command()
            joint_list, prev_joint_list = mime.person_to_baxter(coordinates[0],coordinates[3],coordinates[1],coordinates[2],coordinates[7],coordinates[6],coordinates[8],listener,
                        limb_1, prev_joint_list, command, **joint_list)
            joint_list, prev_joint_list = mime.person_to_baxter(coordinates[3],coordinates[0],coordinates[4],coordinates[5],coordinates[7],coordinates[6],coordinates[8],listener,
                        limb_2,prev_joint_list, command,**joint_list)
            #print("The value of joint_list after call is: ", joint_list)
            print("The value of command in while loop is: ", command)
            mime.move_joints(limb_1,**joint_list)
    except rospy.ROSInterruptException:
        pass
   
