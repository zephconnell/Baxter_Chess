#!/usr/bin/env python

"""
ROS IMPORTS
""" 
#this is a pure Python client library for ROS. It allows programmers using python
# to interface with ROS
import rospy
#cv_bridge is the interface between ROS and openCV.
#here is the web address for the tutorial:
#  http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython 
# you will find tutorials on how to convert OpenCV images to ROS sensor_msgs/Image messages
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
# read about tf (transform) here: http://wiki.ros.org/tf/Tutorials
# there is a tf2 now....program works fine with tf
# here is the info for tf2 should you decide you want to use it: http://wiki.ros.org/tf2
# The web page also contains links to tutorials. 
#I put some formulas to convert back and forth between Euler and Quaternions at the bottom of the page
import tf
# remember that ROS uses topics and streams the data in messages (msgs).  
# this program subscribes to the topic: /cameras/left_hand_camera/image (assuming you are using the left hand camera)
# If you type in cd ros_ws, then get into Baxter's shell by typing ./baxter.sh and then finally type
# rostopic info /cameras/left_hand_camera/image .....it will display that the message type for
# the topic /cameras/left_hand_camera/image is sensor_msgs/Image.  
# this is the reason you need to import the sensor_msgs/Image
from sensor_msgs.msg import Image
# this program uses other ROS message types, particulary, when 
# the ik service is used.
# Here is a link to the geometry_msgs so that you can see all
# the message types.  These message types are used particularly
# when doing transformations. 
# http://wiki.ros.org/geometry_msgs
# if you click on any of the message types at the website, the
# message definition will be displayed. 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
# information on standard message types are found here:
#http://wiki.ros.org/std_msgs
# the header will be used to build pose_stamped msg sent to 
# Baxter's ik service which is called
# /ExternalTools/left/PositionKinematicsNode/IKService
# if you were to type in rosservice info /ExternalTools/left/PositionKinematicsNode/IKService
# it would show you that the msg type is baxter_core_msgs/SolvePositionIK
# and that one of the arguments is a pose_stamped msg.  (which needs a header)
# The Empty message is used to ensure that the simulator is ready to go.
# It is from the line of code: rospy.wait_for_message("/robot/sim/started", Empty)
# The simulator has a topic called "/robot/sim/started" which puts out an empty message
#    once it is ready to go. You would not need this for the real robot.
from std_msgs.msg import (
    Header,
    Empty,
)

# common ros msgs including service (srv) http://wiki.ros.org/common_msgs
import std_srvs.srv

#this allows one to query about ros packages
# in this program it is used to find the path to image files which
# are used to reset Baxter's LCD "face" during the shut down process
import rospkg

#These are the gazebo_msgs needed to spawn and delete models
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

"""
PYTHON IMPORTS
"""
#the next two imports are for the OpenCV libraries. 
"""
This example code uses legacy python code for open CV found here:
https://docs.opencv.org/2.4/modules/core/doc/old_basic_structures.html?highlight=createimage#cv.CreateImage
As you get more familiar with Open CV, you may wish to transition to updated versions. (opencv 3.0)
Just be aware that there may be package dependency as well as compatibility issues with cv_bridge and
move cautiously. 
"""
import cv;
import cv2;
#opencv images are converted to and from Numpy arrays
import numpy
#python module for basic math functions.  In this program you need it for the sqrt()
# in the function called ball_iterate
import math
# python module contains a function that allows the user to map a path
# to a folder.  You will see it just below....os.getenv("HOME") + "/Golf/"
import os

#imports a module that contains a function exit() which
#is used to exit python when there is an exception.
# I commented out the line of code that uses it in the baxter_ik_service method
#   because I did not want the program to exit when an ik solution was not found. 
#   For right now, I have it just printing that no solution was found.  
import sys

#imports a module which has several string classes and useful functions
import string


import random

# in python mutable objects are lists and dictionaries. 
#say you create list1 and then assign list2 = list1.  
# if you change list2[0] to another value, you may find that list1[0] has
# also been changed. Please be careful with python mutable objects.
# I have had programs with unexpected results because I was changing data 
# without realizing it. Read about it here: http://www.geeksforgeeks.org/copy-python-deep-copy-shallow-copy
import copy

""" 
RETHINK IMPORTS
"""

import baxter_interface
 
from baxter_interface import CHECK_VERSION

#These are baxter's core_msgs that allow use of
#  the IK service
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

""" 

"""


image_directory = os.getenv("HOME") + "/Golf/"

#End of header with all the imports
###############################################################


# Locate class
class Locate():
    def __init__(self, arm, distance):
        global image_directory
        # arm ("left" or "right")
        self.limb           = arm
        self._rp = rospkg.RosPack()
        #my package is called test.  You need to change this to the name of your package.
        self._images = (self._rp.get_path('baby_steps') + '/share/images')
        self.limb_interface = baxter_interface.Limb(self.limb) 
        self._joint_names = self.limb_interface.joint_names()       
        print("Getting robot state.... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        
        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)

        # image directory
        self.image_dir = image_directory

        # flag to control saving of analysis images
        # used in canny_it function
        self.save_images = True

        # this is borrowed from the pick and place demo from rethink for the simulator
        self._hover_distance = .15

        # required position accuracy in metres
        self.ball_tolerance = 0.005
        self.tray_tolerance = 0.05

 

        # An orientation for gripper fingers to be overhead and parallel to the obj
        # this orientation was "borrowed" from the baxter_simulator example pick and place
        self.overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

        self.starting_right_joint_angles = {'right_w0': 0.38,
                                  'right_w1': 1.18,
                                  'right_w2': 1.97,
                                  'right_e0': .71,
                                  'right_e1': 2.42,
                                  'right_s0': -0.86,
                                  'right_s1': -2.13}
							  
        
        # number of balls found
        #self.balls_found = 0

        # start positions
        self.ball_tray_x = 0.50                        # x     = front back
        self.ball_tray_y = 0.30                        # y     = left right ----positive for y is to your left as you face forward
        self.ball_tray_z = 0.15                        # z     = up down
        self.golf_ball_x = 0.50                        # x     = front back ---- positive for x is forward--negative x is back behind you 0.50
        self.golf_ball_y = 0.00                        # y     = left right 
        self.golf_ball_z = 0.15                        # z     = up down  
        self.roll        = -1.0 * math.pi              # roll  = horizontal
        self.pitch       = 0.0 * math.pi               # pitch = vertical
        self.yaw         = 0.0 * math.pi               # yaw   = rotation

        #self.pose = [self.golf_ball_x, self.golf_ball_y, self.golf_ball_z,     \
                     #self.roll, self.pitch, self.yaw]
					 #using values from ik_test.py
        self.pose = [self.golf_ball_x, self.golf_ball_y, self.golf_ball_z,     \
                     self.overhead_orientation]

        # camera parameters (NB. other parameters in open_camera)****the description for how this was calculated is on the website
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        self.width        =  800 #960                        # Camera resolution
        self.height       = 800 #600


        # Hough circle accumulator threshold and minimum radius. These were commented out since Hough circle not being used now.
        #self.hough_accumulator = 35
        #self.hough_min_radius  = 13
        #self.hough_max_radius  = 35

        # canny image-------creates an image header 
        # parameters: size(image width and height).  Here we are using Baxter's hand camera resolutions.  
        # next is the bit depth of image elements. Most OpenCV functions use mono8 or bgr8.  Here we use 8.
        # Number of channels per pixel.  Most OpenCV functions support 1-4 channels.  We are using 1 channel. 
        self.canny = cv.CreateImage((self.width, self.height), 8, 1)
        """
        Canny uses two thresholds--an upper and lower.  If a pixel gradient is higher than the upper threshold,
        the pixel is accepted as an edge.  If a pixel gradient value is below the lower threshold, then it is rejected. 
         If the pixel gradient is between the two thresholds, then it will be accepted only if it is connected to
         a pixel that is above the upper threshold. 
         Recommended upper:lower ration between 2:1 and 3:1
        """
        # Canny transform parameters
        self.canny_low  = 45
        self.canny_high = 150

        # minimum ball tray area
        self.min_area = 20000

        # callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)

        # colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # ball tray corners
        self.ball_tray_corner = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # ball tray places
        # you may choose to use these in the future for the board squares
        self.ball_tray_place = [(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),
                                (0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0)]

        # Enable the actuators
        #baxter_interface.RobotEnable().enable()
        
        # set speed as a ratio of maximum speed -- the default is 0.3.  Use this
        #self.limb_interface.set_joint_position_speed(0.5)
        #self.other_limb_interface.set_joint_position_speed(0.5)

        # create image publisher to head monitor
        # enable 'latching on the connect means the last message published is saved and sent to any
        # future subscribers that connect.  This is useful for slow-changing or static data like a map. 
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
		
        # check if gripper error, if yes, reset
        # check if calibrated, if not calibrate the gripper
		# grippers do not open and close unless they are calibrated.

        if self.gripper.error():
            self.gripper.reset();
            print("There was a gripper error, needed reset")
        if(not self.gripper.calibrated()):
            self.gripper.calibrate() 
            print("Calibrated the left gripper") 

        # display the start splash screen
        self.splash_screen("Chess", "Let's Play")

        """
        You can access Baxter's two hand cameras and the head camera using the standard ROS image types and image_transport mechanism
        listed below. You can use the ROS Services to open, close, and configure each of the cameras. See the Camera Control Example
        and Using the Cameras for more information on using the cameras. Useful tools for using cameras in ROS include rviz and the
        image_view program. IMPORTANT: You can only have two cameras open at a time at standard resolutions, due to bandwidth limitations.
         The hand cameras are opened by default at boot-time, 
        
        
        IN THE SIMULATOR, NONE OF THE CAMERA SERVICES WORK--THEREFORE THEY ARE ALL COMMENTED OUT
        In order to have the resolution for the simulator camera at 960 and 600, I had to modify
        baxter_base.gazebo.xacro file. The default is 800 800.  If you go to the "gazebo reference = "left_hand_camera" within
        the document, you will see where you can change the width and height from 800 800 to 960 600 respectively.
        Here is the path to the file:
        home/ros_ws/baxter_common/baxter_description/urdf/baxter_base/baxter_base.gazebo.xacro
        # reset cameras
        #self.reset_cameras()

        # when the left camera closed, the power was sent to the right and head cameras
        #self.close_camera("left")
        
        #self.close_camera("right")
        #now we close the head camera and the power should go back to the right and 
        # left hand camera
        #self.close_camera("head")

        # open required camera...in our case it is the left_hand_camera.  It should open with
        # a resolution of 960, 600.  open_camera is a function in this program.
        #self.open_camera(self.limb, self.width, self.height)
        """
        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # distance of arm to table and ball tray
        self.distance      = distance
        #this will be the height down to the board once it is made and depending
        # on how thick it is.  Remember distances are in meters. 
        self.tray_distance = distance - 0.075  
       

        # move other arm out of harms way
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))



    def set_neutral(self):
        print("Moving to neutral pose...")
        self.limb_interface.move_to_neutral()
        self.other_limb_interface.move_to_neutral()

    def reset(self):
      
        print('Resetting picture')
        print("Picture reset")
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        self.pub.publish(msg)

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self.set_neutral()
        rospy.sleep(1)
        self.reset()
        self.gripper.open()
        
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        else:
            print("Need to disable robot...")
            self._rs.disable()
        return True

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    #this function needs rework to update
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam1 = baxter_interface.camera.CameraController("head_camera")
            cam1.close()
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera--comment the next line out because this automatically opens the head camera if 
        # you close either the right or left hand camera
        #cam.close()

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()

    # close a camera
    def close_camera(self, camera):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
        x = (self.width / 2)                                                         \
          + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2)                                                        \
          + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)

    # Not a tree walk due to python recursion limit
    def tree_walk(self, image, x_in, y_in):
        almost_black = (1, 1, 1)

        pixel_list = [(x_in, y_in)]                   # first pixel is black save position
        cv.Set2D(image, y_in, x_in, almost_black)     # set pixel to almost black
        to_do = [(x_in, y_in - 1)]                    # add neighbours to to do list
        to_do.append([x_in, y_in + 1])
        to_do.append([x_in - 1, y_in])
        to_do.append([x_in + 1, y_in])

        while len(to_do) > 0:
            x, y = to_do.pop()                             # get next pixel to test
            if cv.Get2D(image, y, x)[0] == self.black[0]:  # if black pixel found
                pixel_list.append([x, y])                  # save pixel position
                cv.Set2D(image, y, x, almost_black)        # set pixel to almost black
                to_do.append([x, y - 1])                   # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

        return pixel_list

    # Remove artifacts and find largest object
    def look_for_ball_tray(self, canny):
        width, height = cv.GetSize(canny)

        centre   = (0, 0)
        max_area = 0

        # for all but edge pixels
        for x in range(1, width - 2):
            for y in range(1, height - 2):
                if cv.Get2D(canny, y, x)[0] == self.black[0]:       # black pixel found
                    pixel_list = self.tree_walk(canny, x, y)        # tree walk pixel
                    if len(pixel_list) < self.min_area:             # if object too small
                        for l in pixel_list:
                            cv.Set2D(canny, l[1], l[0], self.white) # set pixel to white
                    else:                                           # if object found
                        n = len(pixel_list)
                        if n > max_area:                            # if largest object found
                            sum_x  = 0                              # find centre of object
                            sum_y  = 0
                            for p in pixel_list:
                                sum_x  = sum_x + p[0]
                                sum_y  = sum_y + p[1]

                            centre = sum_x / n, sum_y / n           # save centre of object
                            max_area = n                            # save area of object

        if max_area > 0:                                            # in tray found
            cv.Circle(canny, (centre), 9, (250, 250, 250), -1)      # mark tray centre

        # display the modified canny
        cv.ShowImage("Modified Canny", canny)

        # 3ms wait
        cv.WaitKey(3)
        return centre                                        # return centre of object

    # flood fill edge of image to leave objects
    def flood_fill_edge(self, canny):
        width, height = cv.GetSize(canny)

        #(name of array, row, column, value)
        # set boarder pixels to white
        for x in range(width):
            cv.Set2D(canny, 0, x, self.white)
            cv.Set2D(canny, height - 1, x, self.white)

        for y in range(height):
            cv.Set2D(canny, y, 0, self.white)
            cv.Set2D(canny, y, width - 1, self.white)

        # prime to do list
        to_do = [(2, 2)]
        to_do.append([2, height - 3])
        to_do.append([width - 3, height - 3])
        to_do.append([width - 3, 2])

        while len(to_do) > 0:
            x, y = to_do.pop()                               # get next pixel to test
            if cv.Get2D(canny, y, x)[0] == self.black[0]:    # if black pixel found
                cv.Set2D(canny, y, x, self.white)            # set pixel to white
                to_do.append([x, y - 1])                     # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

    # camera call back function
    # consider changing the callback function to only take the data. 
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            # using ROS cv_bridge to convert the image to a CV image. 
            # go to the web page: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
            # there is a tutorial there
            # data is the image that is coming from the rostopic via the hand_camera
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait --wait for 3 milliseconds
        cv.WaitKey(3)

    # left camera call back function
    #not sure why we need the second parameter
    # consider changing camera_callback function so that it only takes just the data
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")

    # right camera call back function
    def right_camera_callback(self, data):
        self.camera_callback(data, "Right Hand Camera")

    # head camera call back function
    def head_camera_callback(self, data):
        self.camera_callback(data, "Head Camera")

    # create subscriber to the required camera
    #this is one of Baxter's topic. When a topic is subscribed to
    # there must be a callback function.  
    def subscribe_to_camera(self, camera):
        if camera == "left":
            callback = self.left_camera_callback
            camera_str = "/cameras/left_hand_camera/image"
        elif camera == "right":
            callback = self.right_camera_callback
            camera_str = "/cameras/right_hand_camera/image"
        elif camera == "head":
            callback = self.head_camera_callback
            camera_str = "/cameras/head_camera/image"
        else:
            sys.exit("ERROR - subscribe_to_camera - Invalid camera")

        camera_sub = rospy.Subscriber(camera_str, Image, callback)

    # Convert cv image to a numpy array
    def cv2array(self, im):
        depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
  
        arrdtype=im.depth
        a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        a.shape = (im.height, im.width, im.nChannels)

        return a
    """
 
    """
    #the original code imported conversions from moveit commander in the header.
    # This caused an error only on shut down---did not affect the program.  Rather than deal with errors
    # even at shut down while you are trying to implement code, I went to 
    # web site below and copied and pasted the two needed function here with
    # slight modification only in the except line where you print "Unexpected error"
    # https://github.com/ros-planning/moveit/blob/kinetic-devel/moveit_commander/src/moveit_commander/conversions.py 
    # These functions create the PoseStamped msgs needed for the ik service. 
    def list_to_pose(self, pose_list):
        pose_msg = Pose()
        try:
            if len(pose_list) == 7:
                pose_msg.position.x = pose_list[0]
                pose_msg.position.y = pose_list[1]
                pose_msg.position.z = pose_list[2]
                pose_msg.orientation.x = pose_list[3]
                pose_msg.orientation.y = pose_list[4]
                pose_msg.orientation.z = pose_list[5]
                pose_msg.orientation.w = pose_list[6]
            elif len(pose_list) == 6: 
                pose_msg.position.x = pose_list[0]
                pose_msg.position.y = pose_list[1]
                pose_msg.position.z = pose_list[2]
                q = tf.transformations.quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
                pose_msg.orientation.x = q[0]
                pose_msg.orientation.y = q[1]
                pose_msg.orientation.z = q[2]
                pose_msg.orientation.w = q[3]
        except:
            print "Unexpected error:", sys.exc_info()[0]
            raise 
        #uncomment if you want to print the pose message to see what is sent to baxter_ik_move 
        #print("*********************") 
        #print pose_msg
        return pose_msg

    def list_to_pose_stamped(self, pose_list, target_frame):
        pose_msg = PoseStamped()
        pose_msg.pose = self.list_to_pose(pose_list)
        pose_msg.header.frame_id = target_frame
        pose_msg.header.stamp = rospy.Time.now()
        return pose_msg


    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = self.list_to_pose_stamped(rpy_pose, "base")
        #quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            self.splash_screen("Valid", "move")
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            print("Requested move was not valid")
            #this will try 3 more times to see if the requested move has a valid solution
            if limb == 'left':  
                # requesting the starting position again for this program--will need a better solution
                self.pose = (self.ball_tray_x,
                             self.ball_tray_y,
                             self.ball_tray_z,
                             self.roll, self.pitch, self.yaw)
                self.baxter_ik_move(self.limb, self.pose)
                #return False
                #sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")
            else:
                #self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
                print("moving right arm out of the way")
                self.other_limb_interface.move_to_joint_positions(self.starting_right_joint_angles)
                
         
        
        if self.limb == limb:               # if working arm
            """
            The package baxter_interface is found at api.rethinkrobotics.com/baxter_interface/html/index.html
            In the limb class, the functions are described.
            endpoint_pose() returns the Cartesian endpoint pose {position, orientation}. tf (transform frame) of the gripper
                             All measurements reported for the endpoint(pose, twist, and wrench) are with respect to the /base frame
                             of the robot.
            pose = {'position': (x,y,z), 'orientation': (x,y,z,w)}
            Therefore, for the line of code that says position = quaternion_pose['position'],
            this means that position = [(x,y,z)] of the endpoint_pose
            """
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            # if Baxter's left arm reached the coordinates over the board that we want,
            # we are saving those x and y positions. This is why we are using the x and y
            # from position above. 
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]


    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)

    # used to place camera over the ball tray
    def ball_tray_iterate(self, iteration, centre):
        # print iteration number
        print "Egg Tray Iteration ", iteration

        # find displacement of object from centre of image
        pixel_dx    = (self.width / 2) - centre[0]
        pixel_dy    = (self.height / 2) - centre[1]
        pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
        error       = float(pixel_error * self.cam_calib * self.tray_distance)
        print("Here is the error: ", error)
        print("Here is the self.tray_tolerance: ", self.tray_tolerance)

        x_offset = - pixel_dy * self.cam_calib * self.tray_distance
        y_offset = - pixel_dx * self.cam_calib * self.tray_distance

        # if error in current position too big
        
        if error > self.tray_tolerance:
            # correct pose
            self.update_pose(x_offset, y_offset)
            # find new centre
            centre = self.canny_it(iteration)

            # find displacement of object from centre of image
            pixel_dx    = (self.width / 2) - centre[0]
            pixel_dy    = (self.height / 2) - centre[1]
            pixel_error = math.sqrt((pixel_dx * pixel_dx) + (pixel_dy * pixel_dy))
            error       = float(pixel_error * self.cam_calib * self.tray_distance)

        return centre, error

    # randomly adjust a pose to dither arm position
    # used to prevent stalemate when looking for ball tray
    # the numbers generated will be floats between 0.0 to 1.0
    def dither(self):
        x = self.ball_tray_x
        y = self.ball_tray_y + (random.random() / 10.0)
        pose = (x, y, self.ball_tray_z, self.roll, self.pitch, self.yaw)

        return pose

    # find the ball tray
    def canny_it(self, iteration):
        if self.save_images:
            # save raw image of ball tray
            file_name = self.image_dir + "ball_tray_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, cv.fromarray(self.cv_image))
            width, height = cv.GetSize(cv.fromarray(self.cv_image))
            print("Here is the width: {0} and here is the height: {1}.".format(width, height))

        # create an empty image variable, the same dimensions as our camera feed.
        gray = cv.CreateImage((cv.GetSize(cv.fromarray(self.cv_image))), 8, 1)

        # convert the image to a grayscale image
        cv.CvtColor(cv.fromarray(self.cv_image), gray, cv.CV_BGR2GRAY)

        # display image on head monitor
        font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
        position = (30, 60)
        cv.PutText(cv.fromarray(self.cv_image), "Looking for ball tray", position, font, self.white)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(self.cv_image, encoding="bgr8")
        self.pub.publish(msg)


        #create a Trackbar for user to enter threshold
        #The info can be obtained on github by Atlas7/opencv_python_tutorials. 
        # he shows how to create a tracker bar to adjust the greyscale thresholds
        # http://mathalope.co.uk/2015/06/03/canny-edge-detection-app-with-opencv-python/
        # create a canny edge detection map of the greyscale image
        cv.Canny(gray, self.canny, self.canny_low, self.canny_high, 3)
        
        # display the canny transformation
        cv.ShowImage("Canny Edge Detection", self.canny)

        if self.save_images:
            # save Canny image of ball tray
            file_name = self.image_dir + "canny_tray_" + str(iteration) + ".jpg"
            cv.SaveImage(file_name, self.canny)

        # flood fill edge of image to leave only objects
        self.flood_fill_edge(self.canny)
        ball_tray_centre = self.look_for_ball_tray(self.canny)

        # 3ms wait
        cv.WaitKey(3)

        while ball_tray_centre[0] == 0:
            if random.random() > 0.6:
                self.baxter_ik_move(self.limb, self.dither())

            ball_tray_centre = self.canny_it(iteration)
            print("Here are the coordinates of ball_tray_centre: ", ball_tray_centre)

        return ball_tray_centre

    # find places for golf balls
    def find_places(self, c):
        # find long side of ball tray
        # this was commented out because you are using a square
        #l1_sq = ((c[1][0] - c[0][0]) * (c[1][0] - c[0][0])) +           \
                #((c[1][1] - c[0][1]) * (c[1][1] - c[0][1]))
        #l2_sq = ((c[2][0] - c[1][0]) * (c[2][0] - c[1][0])) +           \
                #((c[2][1] - c[1][1]) * (c[2][1] - c[1][1]))

        #if l1_sq > l2_sq:                     # c[0] to c[1] is a long side
            #cc = [c[0], c[1], c[2], c[3]]
        #else:                                 # c[1] to c[2] is a long side
            #cc = [c[1], c[2], c[3], c[0]][0]

        cc = [c[0], c[1], c[2], c[3]]
        
        # ball tray corners in baxter coordinates
        for i in range(4):
            self.ball_tray_corner[i] = self.pixel_to_baxter(cc[i], self.tray_distance)

        # ball tray places in pixel coordinates
        ref_x = cc[0][0]
        ref_y = cc[0][1]
        dl_x  = (cc[1][0] - cc[0][0]) / 8
        dl_y  = (cc[1][1] - cc[0][1]) / 8
        ds_x  = (cc[2][0] - cc[1][0]) / 8  # was 6
        ds_y  = (cc[2][1] - cc[1][1]) / 8  # was 6
        #please see the diagram with the map of the functions location and how
        # they corrolate to the colors.  The order below is different than that of
        # the original. 
        p     = {}
        p[0]  = (ref_x + (7 * dl_x) + (1 * ds_x), ref_y + (7 * dl_y) + (1 * ds_y))
        p[1]  = (ref_x + (7 * dl_x) + (3 * ds_x), ref_y + (7 * dl_y) + (3 * ds_y))
        p[2]  = (ref_x + (7 * dl_x) + (5 * ds_x), ref_y + (7 * dl_y) + (5 * ds_y))
        p[3]  = (ref_x + (7 * dl_x) + (7 * ds_x), ref_y + (7 * dl_y) + (7 * ds_y))
        p[4]  = (ref_x + (5 * dl_x) + (1 * ds_x), ref_y + (5 * dl_y) + (1 * ds_y))
        p[5]  = (ref_x + (5 * dl_x) + (3 * ds_x), ref_y + (5 * dl_y) + (3 * ds_y))
        p[6]  = (ref_x + (5 * dl_x) + (5 * ds_x), ref_y + (5 * dl_y) + (5 * ds_y))
        p[7]  = (ref_x + (5 * dl_x) + (7 * ds_x), ref_y + (5 * dl_y) + (7 * ds_y))
        p[8]  = (ref_x + (3 * dl_x) + (1 * ds_x), ref_y + (3 * dl_y) + (1 * ds_y))
        p[9]  = (ref_x + (3 * dl_x) + (3 * ds_x), ref_y + (3 * dl_y) + (3 * ds_y))
        p[10] = (ref_x + (3 * dl_x) + (5 * ds_x), ref_y + (3 * dl_y) + (5 * ds_y))
        p[11] = (ref_x + (3 * dl_x) + (7 * ds_x), ref_y + (3 * dl_y) + (7 * ds_y))
        #added these extra rows
        p[12] = (ref_x + (1 * dl_x) + (1 * ds_x), ref_y + (1 * dl_y) + (1 * ds_y))
        p[13] = (ref_x + (1 * dl_x) + (3 * ds_x), ref_y + (1 * dl_y) + (3 * ds_y))
        p[14] = (ref_x + (1 * dl_x) + (5 * ds_x), ref_y + (1 * dl_y) + (5 * ds_y))
        p[15] = (ref_x + (7 * dl_x) + (7 * ds_x), ref_y + (1 * dl_y) + (1 * ds_y))

        for i in range(16):
            # mark position of ball tray places ---different colors were used so you could correlate what the calculations
            #   above are doing. 
            #parameters are img, center, radius, circle color(bgr for lime green is (0,250,0)), -1 means that it will fill the shape)
            if i >= 0  and i <= 3:
                cv.Circle(cv.fromarray(self.cv_image), (int(p[i][0]), int(p[i][1])), 5, (0, 255, 0), -1) 
                # ball tray places in baxter coordinates
                self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)
            if i > 3 and i <= 7:
                cv.Circle(cv.fromarray(self.cv_image), (int(p[i][0]), int(p[i][1])), 5, (0, 0, 255), -1) #red in brg is (0, 0, 255)
                # ball tray places in baxter coordinates
                self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)
            if i > 7 and i <= 11:
                cv.Circle(cv.fromarray(self.cv_image), (int(p[i][0]), int(p[i][1])), 5, (255, 0, 0), -1)  #blue in brg is (255, 0, 0)
                # ball tray places in baxter coordinates
                self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)
            if i > 11 and i <= 15:
                cv.Circle(cv.fromarray(self.cv_image), (int(p[i][0]), int(p[i][1])), 5, (255, 0, 255), -1) #purple/pink in brg is (255, 0, 255)
                # ball tray places in baxter coordinates
                self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)
                

            # convert ball tray places in baxter coordinates  --this uses the formula from the web page: 
            # http://sdk.rethinkrobotics.com/wiki/Worked_Example_Visual_Servoing
             
            self.ball_tray_place[i] = self.pixel_to_baxter(p[i], self.tray_distance)
            
           
        # display the ball tray places
        cv.ShowImage("Egg tray", cv.fromarray(self.cv_image))

        if self.save_images:
            # save ball tray image with overlay of ball tray and ball positions
            file_name = self.image_dir + "ball_tray.jpg"
            cv.SaveImage(file_name, cv.fromarray(self.cv_image))

        # 3ms wait
        cv.WaitKey(3)

    # find four corners of the ball tray
    def find_corners(self, centre):
        self.center = centre
        # find bottom corner
        max_x  = 0
        max_y  = 0

        for x in range(100, self.width -100):
        #for x in range(100, self.width - 100):
            y = self.height - 20
            #(array, index1, index2)
            while y > 0 and cv.Get2D(self.canny, y, x)[0] > 100:
            #while y > 0 and cv.Get2D(self.canny, y, x)[0] > 100:
                y = y - 1
            if y > 20:
                cv.Set2D(cv.fromarray(self.cv_image), y, x, (255, 0, 0)) #red is bgr(0,0,255)
                if y > max_y:
                    max_x = x
                    max_y = y

        corner_1 = (max_x, max_y)

        # find left corner
        min_x  = self.width
        min_y  = 0

        for y in range(100, self.height - 100):
            x = 20
            while x < self.width - 1 and cv.Get2D(self.canny, y, x)[0] > 100:
                x = x + 1
            if x < self.width - 20:
                cv.Set2D(cv.fromarray(self.cv_image), y, x, (0, 255, 0, 0))
                if x < min_x:
                    min_x = x
                    min_y = y

        corner_2 = (min_x, min_y)

        # display corner image
        cv.ShowImage("Corner", cv.fromarray(self.cv_image))

        if self.save_images:
            # save Canny image
            file_name = self.image_dir + "egg_tray_canny.jpg"
            cv.SaveImage(file_name, self.canny)

            # mark corners and save corner image
            cv.Circle(cv.fromarray(self.cv_image), corner_1, 9, (0, 0, 255), -1)
            cv.Circle(cv.fromarray(self.cv_image), corner_2, 9, (0, 250, 0), -1)
            file_name = self.image_dir + "corner.jpg"
            cv.SaveImage(file_name, cv.fromarray(self.cv_image))

        # 3ms wait
        cv.WaitKey(3)

        # two corners found and centre known find other two corners
        corner_3 = ((2 * centre[0]) - corner_1[0], (2 * centre[1]) - corner_1[1])
        corner_4 = ((2 * centre[0]) - corner_2[0], (2 * centre[1]) - corner_2[1])

        if self.save_images:
            # save Canny image
            file_name = self.image_dir + "egg_tray_canny.jpg"
            cv.SaveImage(file_name, self.canny)

            # mark corners and save corner image
            cv.Circle(cv.fromarray(self.cv_image), corner_1, 9, (0, 0, 255), -1)
            cv.Circle(cv.fromarray(self.cv_image), corner_2, 9, (0, 250, 0), -1)
            cv.Circle(cv.fromarray(self.cv_image), corner_3, 9, (255, 0, 0), -1)
            cv.Circle(cv.fromarray(self.cv_image), corner_4, 9, (255, 250, 0), -1)
            
            file_name = self.image_dir + "all_four_corners.jpg"
            cv.SaveImage(file_name, cv.fromarray(self.cv_image))

        # draw ball tray boundry
        c1 = (int(corner_1[0]), int(corner_1[1]))
        c2 = (int(corner_2[0]), int(corner_2[1]))
        c3 = (int(corner_3[0]), int(corner_3[1]))
        c4 = (int(corner_4[0]), int(corner_4[1]))

        cv.Line(cv.fromarray(self.cv_image), c1, c2, (255, 0, 0), thickness=3)
        cv.Line(cv.fromarray(self.cv_image), c2, c3, (0, 255, 0), thickness=3)
        cv.Line(cv.fromarray(self.cv_image), c3, c4, (255, 255, 0), thickness=3)
        cv.Line(cv.fromarray(self.cv_image), c4, c1, (0, 0, 255), thickness=3)

        return True, (corner_1, corner_2, corner_3, corner_4)

    # find the ball tray
    def find_ball_tray(self):
        #define and intialize boolean variable named ok
        ok = False
        #while ok is false
        while not ok:
            ball_tray_centre = self.canny_it(0)

            error     = 2 * self.tray_tolerance
            iteration = 1

            # iterate until arm over centre of tray
            while error > self.tray_tolerance:
                ball_tray_centre, error = self.ball_tray_iterate(iteration,       \
                                          ball_tray_centre)
                iteration              += 1

            # find ball tray corners in pixel units
            (ok, corners) = self.find_corners(ball_tray_centre)
        
        self.find_places(corners)
    """
 
    """

    # display message on head display
    def splash_screen(self, s1, s2):
        splash_array = numpy.zeros((self.height, self.width, 3), numpy.uint8)
        font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 3.0, 3.0, 9)

        ((text_x, text_y), baseline) = cv.GetTextSize(s1, font)
        org = ((self.width - text_x) / 2, (self.height / 3) + (text_y / 2))
        cv2.putText(splash_array, s1, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        ((text_x, text_y), baseline) = cv.GetTextSize(s2, font)
        org = ((self.width - text_x) / 2, ((2 * self.height) / 3) + (text_y / 2))
        cv2.putText(splash_array, s2, org, cv.CV_FONT_HERSHEY_SIMPLEX, 3.0,          \
                    self.white, thickness = 7)

        splash_image = cv.fromarray(splash_array)

        # 3ms wait
        cv2.waitKey(3)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(splash_array, encoding="bgr8")
        self.pub.publish(msg)

    def gripper_open(self):
        self.gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self.gripper.close()
        rospy.sleep(1.0)


    def _approach(self, pose):
        pose1 = copy.deepcopy(pose)
        pose1[2] = pose1[2] + self._hover_distance

        # approach with a pose the hover-distance above the requested pose
        self.baxter_ik_move(self.limb, pose1)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self.limb_interface.endpoint_pose()
        ik_pose = []
        ik_pose.append(current_pose['position'].x) 
        ik_pose.append(current_pose['position'].y) 
        ik_pose.append(current_pose['position'].z + self._hover_distance)
        ik_pose.append(current_pose['orientation'].x) 
        ik_pose.append(current_pose['orientation'].y) 
        ik_pose.append(current_pose['orientation'].z) 
        ik_pose.append(current_pose['orientation'].w)
        self.baxter_ik_move(self.limb, ik_pose)
        # servo up from current pose
        

    def _servo_to_pose(self, pose):
        # servo down to release
        self.baxter_ik_move(self.limb, pose)
        

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()


    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

#END OF CLASS LOCATOR   
#########################################################

# read the setup parameters from setup.dat
#This is the first function to look at.It is entirely
# optional.  The limb and distance can be "hard coded" or 
#you can even get user input if desired. 
def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: golf_setup must be run before golf")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

#this function loads the models used in gazebo
def load_gazebo_models(table_pose=Pose(position=Point(x= .75, y=0.2, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6, y=0.25, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baby_steps')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/chessboard.sdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


if __name__ == "__main__":
    #all scripts need to initalize a node.  Remember that a node in ROS is an executable that uses
    # ROS to communicate with other nodes.  anonymous = True adds a time stamp to the name so that
    #it is highly unlikely that two nodes will have the same name. If two nodes have the same name,
    # the oldest node will be "killed"
    rospy.init_node("pick_and_place", anonymous = True)
    #load_gazebo_models()
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)
    #Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # get setup parameters---go to function with #2 by it
    limb, distance = get_setup()
    print "limb     = ", limb
    print "distance = ", distance
    # create an instance of the class Locate. 
    locator = Locate(limb, distance)
    rospy.on_shutdown(locator.clean_shutdown) 
    raw_input("Press Enter to start: ")
      
    locator.gripper.open()
    

    # move the arm whose camera you are using close to the board
    # you may wish to change this starting location. 
    locator.pose = [locator.ball_tray_x,
                    locator.ball_tray_y,
                    locator.ball_tray_z,
                    locator.roll, locator.pitch, locator.yaw]
    locator.baxter_ik_move(locator.limb, locator.pose)

    # find the ball tray
    locator.find_ball_tray()

    # going to use some of the pick and place code from the baxter simulator ik demo
    #  to have Baxter move his arms to all the corners. 
    print("**************************************")
    print("We are now going to move to all the corners and pick and place")
    print('\n' * 3)
    board_corners = list() 
    
    locator.pose = [copy.copy(locator.ball_tray_corner[0][0]),
                    copy.copy(locator.ball_tray_corner[0][1]),
                    locator.golf_ball_z - .25,
                    locator.roll,
                    locator.pitch,
                    locator.yaw]
    
    board_corners.append(locator.pose)

    locator.pose = [copy.copy(locator.ball_tray_corner[1][0]),
                    copy.copy(locator.ball_tray_corner[1][1]),
                    locator.golf_ball_z -.25,
                    locator.roll,
                    locator.pitch,
                    locator.yaw]
    
    board_corners.append(locator.pose)

    locator.pose = [copy.copy(locator.ball_tray_corner[2][0]),
                    copy.copy(locator.ball_tray_corner[2][1]),
                    locator.golf_ball_z -.25,
                    locator.roll,
                    locator.pitch,
                    locator.yaw]
    
    board_corners.append(locator.pose)

    locator.pose = [copy.copy(locator.ball_tray_corner[3][0]),
                    copy.copy(locator.ball_tray_corner[3][1]),
                    locator.golf_ball_z -.25,
                    locator.roll,
                    locator.pitch,
                    locator.yaw]
    
    board_corners.append(locator.pose)
    #locator.baxter_ik_move(locator.limb, locator.pose)
    count = 0
    while count < len(board_corners):
        print('\n' *3)
        print("\nPicking...")
        locator.pick(board_corners[count])
        print("\nPlacing...")
        if count == 3:
            locator.place(board_corners[count])
            count += 1
        else:
            count += 1
            locator.place(board_corners[count])
    print("All done")
    #program starts the shut down process here
    
