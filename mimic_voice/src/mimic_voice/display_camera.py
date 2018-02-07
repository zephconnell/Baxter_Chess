#!/usr/bin/env python
import baxter_interface
import rospy
from sensor_msgs.msg import Image

from std_msgs.msg import(
    UInt16,
)
import cv2
import cv_bridge
import rospkg

from std_msgs.msg import Header


class Camera(object):

    def __init__(self):
        self._rp = rospkg.RosPack()
        self._images = (self._rp.get_path('mimic_voice') + '/share/images')

    def close_left_hand_camera(self):
        left = baxter_interface.CameraController("left_hand_camera")
        left.close()

    def close_head_camera(self):
        head = baxter_interface.CameraController("head_camera")
        head.close()

    def open_head_camera(self):
        head_open = baxter_interface.CameraController("head_camera")
        head_open.resolution = (1280,800)
        head_open.open()

    def republish(msg):
        #msg = rospy.wait_for_message("/cameras/" + "head_camera" + "/image", Image)
        #display_pub = rospy.Publisher('/robot/xdisplay',Image,queue_size = 10)
        display_pub.publish(msg)

    def display(self):
            display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size = 10)
            self.close_left_hand_camera()
            print('Closed left hand camera')
            self.open_head_camera()
            print('Open head camera')
            def republish(msg):
                display_pub.publish(msg)
            sub = rospy.Subscriber("/cameras/head_camera/image",Image, republish,None,1)
            print('Displaying head camera on screen')

    def reset_picture(self):
        img = cv2.imread(self._images + '/default.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding= "bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size=10)
        pub.publish(msg)

    def clean_shut_down(self):
        self.close_head_camera()
        self.reset_picture()

def main():
    rospy.init_node("show_images")
    pi = Camera()
    rospy.on_shutdown(pi.clean_shut_down)
    
    pi.display()
   
    print('Program complete')

if __name__ =="__main__":
    main()
        
        
        
        
        
        




