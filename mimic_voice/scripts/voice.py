#!/usr/bin/env python


import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import (
      String,
      Float64,
)

import rospkg


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
"""
KEY FOR THE VOICE COMMANDS

Green means rotate the W0 joint inward by 0.4 radians (23 degrees)
Yellow means rotate the W0 joint inward by 0.1 radians(approx. 5 degrees)
Red means rotate the W0 joint outward by 0.4 radians
Orange means rotate the W0 joint outward by 0.1 radians
Blue means rotate the W0 joint by to zero radians

Zero means "twist" the W2 joint to zero radians
One means "twist" the W2 joint inward by 0.4 radians
Two means "twist" the W2 joint inward by 1.57 radians (90 degrees)
Three means "twist" the W2 joint inward by 3.0 radians (171 degrees)
Four means "twist" the W2 joint outward by 1.57 radians (90 degrees)
Five means "twist the W2 joint outward by 3.0 radians (171 degrees)
Six means "twist the W2 joint outward by 0.2 radians

Small means flex the W1 joint 0 radians
Medium means flex the W1 joint 0.4 radians (23 degrees)
Large means flex the W1 joint by 1.57 radians 

Alpha means extend the W1 joint 0 radians
Bravo means extend the W1 joint 0.4 radians (23 degrees)
Charlie means extend the W joint by 1.4 radians (80 degrees)

"""

class Voice:
    def __init__(self):
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print("Enabling robot... ")
        rs.enable()

        self.limb_right = baxter_interface.Limb('right')
        self.limb_left = baxter_interface.Limb('left')
        self.grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.lj = self.limb_left.joint_names()
        self.rj = self.limb_right.joint_names()

        self.paused = False
        self.rate = 5
        self.r = rospy.Rate(self.rate)

        self.right_w0_pub = rospy.Publisher('/right_w0', Float64, queue_size = 10)
        self.right_w1_pub = rospy.Publisher('/right_w1', Float64, queue_size = 10)
        self.right_w2_pub = rospy.Publisher('/right_w2', Float64, queue_size = 10)
    
        self.left_w0_pub = rospy.Publisher('/left_w0', Float64, queue_size = 10)
        self.left_w1_pub = rospy.Publisher('/left_w1', Float64, queue_size = 10)
        self.left_w2_pub = rospy.Publisher('/left_w2', Float64, queue_size = 10) 
       
        self.right_w0 = Float64()
        self.right_w1 = Float64()
        self.right_w2 = Float64()

        self.left_w0 = Float64()
        self.left_w1 = Float64()
        self.left_w2 = Float64()

        self.right_w0 = 0.0
        self.right_w1 = 0.0
        self.right_w2 = 0.0

        self.left_w0 = 0.0
        self.left_w1 = 0.0
        self.left_w2 = 0.0

        self.message = 'sayonara'
        
        if self.grip_left.error():
            self.grip_left.reset()

        if self.grip_right.error():
            self.grip_right.reset()

        if (not self.grip_left.calibrated() and
            self.grip_left.type() != 'custom'):
            self.grip_left.calibrate()

        if (not self.grip_right.calibrated() and
            self.grip_right.type() != 'custom'):
            self.grip_right.calibrate()
        
        # A flag to determine whether or not voice control is paused
        self.paused = False
        

        #publishing the verbal command to the command topic
        self.pub = rospy.Publisher('/command', String, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'right charlie': ['right charlie', 'charlie right'],
                                    'right alpha': ['right alpha', 'alpha right'],
                                    'right bravo': ['right bravo', 'bravo right'],
                                    'right small': ['right small', 'small right'],
                                    'right large': ['right large', 'large right'],
                                    'right medium': ['right mid','right medium','medium right', 'mid right'],
                                    'left charlie': ['left charlie', 'charlie left'],
                                    'left alpha': ['left alpha', 'alpha left'],
                                    'left bravo': ['left bravo', 'bravo left'],
                                    'left small': ['left small', 'small left'],
                                    'left medium': ['left mid','left medium', 'mid left', 'medium left'],
                                    'left large': ['left large', 'large left'],
                                    'right open': ['open right', 'right open'],
                                    'right close': ['close right', 'right close'],
                                    'left open': ['open left', 'left open'],
                                    'left close': ['close left', 'left close'],
                                    'stop':['stop', 'halt', 'stop now','abort', 'kill', 'panic', 'help'],
                                    'start':['start','start now','begin'],
                                    'pause': ['pause speech', 'pause'],
                                    'continue': ['continue speech', 'continue'],
                                    'right green': ['right green', 'green right'],
                                    'right yellow': ['right yellow', 'yellow right'],
                                    'right red': ['right red', 'red right'],
                                    'right orange': ['right orange', 'orange right'],
                                    'right blue': ['right blue', 'blue right'],
                                    'left green': ['left green', 'green left'],
                                    'left yellow': ['left yellow', 'yellow left'],
                                    'left red': ['left red', 'red left'],
                                    'left orange': ['left orange', 'orange left'],
                                    'left blue':['left blue', 'blue left'],
                                    'right zero': ['right zero', 'zero right'],
                                    'right one': ['right one', 'one right'],
                                    'right two': ['right two', 'two right'],
                                    'right three': ['right three', 'three right'],
                                    'right four': ['right four', 'four right'],
                                    'right five': ['right five', 'five right'],
                                    'right six': ['right six', 'six right'],                                    
                                    'left zero': ['left zero','zero left'],
                                    'left one': ['left one', 'one left'],
                                    'left two': ['left two', 'two left'],
                                    'left three': ['left three', 'three left'],
                                    'left four': ['left four', 'four left'],
                                    'left five': ['left five', 'five left'],
                                    'left six': ['left six', 'six left']}

        
        rospy.loginfo("Ready to receive voice commands")          
                   
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
         
    def speech_callback(self, msg):
        
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
        
        # If the user has asked to pause/continue voice control,
        # set the flag accordingly 
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
        
        # If voice control is paused, simply return without
        # performing any action
        if self.paused:
            return       
        
        # The list of if-then statements should be fairly
        # self-explanatory
        
        # open the right gripper                       
        if command == 'right open':  
            self.grip_right.open()

        # close the right gripper            
        elif command == 'right close':
            self.grip_right.close()

        #extend the W1 to zero radians
        elif command == 'right alpha':
            voice.right_w1 = 0.0

        # extend the W1 by 0.2 radians (23 degrees)
        elif command == 'right bravo':
            voice.right_w1 = voice.right_w1 - 0.4

        # extend the W1 by -1.4 radians (80 degrees)
        elif command == 'right charlie':
            voice.right_w1 = -1.4

        elif command == 'right small':
            voice.right_w1 = 0.0

        elif command == 'right medium':
            voice.right_w1 = voice.right_w1 + 0.4
            
        elif command == 'right large':
            voice.right_w1 = 1.57

        elif command == 'left bravo':
            voice.left_w1 = voice.left_w1 - 0.4

        elif command =='left alpha':
            voice.left_w1 = 0.0

        elif command == 'left charlie':
            voice.left_w1 = -1.4
                
                
        elif command == 'left open':  
            self.grip_left.open()
            
        elif command == 'left close':
            self.grip_left.close()

        

        elif command == 'left small':
            voice.left_w1 = 0.0

        elif command == 'left medium':
            voice.left_w1 = voice.left_w1 + 0.4

        elif command == 'left large':
            voice.left_w1 = 1.57 

        elif command == 'right zero':
            voice.right_w2 = 0.0

        #right wrist twists inward by 0.4
        elif command == 'right one':
            voice.right_w2 = voice.right_w2 - 0.4

        elif command == 'right two':
            voice.right_w2 = -1.57

        elif command == 'right three':
            voice.right_w2 = - 3.0

        elif command == 'right four':
            voice.right_w2 = 1.57

        elif command == 'right five':
            voice.right_w2 = 3.0   

        #right wrist twists outward by 0.4
        elif command == 'right six':
            voice.right_w2 = voice.right_w2 + 0.4

        elif command == 'left zero':
            voice.left_w2 = 0.0
  
        #left wrist twists inward by 0.4
        elif command == 'left one':
            voice.left_w2 = voice.left_w2 + 0.4

        elif command == 'left two':
            voice.left_w2 = 1.57

        elif command == 'left three':
            voice.left_w2 = 3.0

        elif command == 'left four':
            voice.left_w2 = - 1.57

        elif command == 'left five':
            voice.left_w2 = - 3.0

        #left wrist twists outward by 0.4
        elif command == 'left six':
            voice.left_w2 = voice.left_w2 - 0.4

        elif command == 'stop':
            voice.message = "stop"
        elif command == 'start':
            voice.message = "start"


        # rotate the right arm inward
        elif command == 'right green': 
            voice.right_w0 = voice.right_w0 - 0.4

        elif command == 'right yellow': 
            voice.right_w0 = voice.right_w0 - 0.1


        # rotate the right arm outward
        elif command == 'right red': 
            voice.right_w0 = voice.right_w0 + 0.4

        elif command == 'right orange': 
            voice.right_w0 = voice.right_w0 + 0.1

        # rotate the right arm back to zero
        elif command == 'right blue': 
            voice.right_w0 = 0.0

        # rotate the left arm inward
        elif command == 'left green': 
            voice.left_w0 = voice.left_w0 + 0.4

        elif command == 'left yellow': 
            voice.left_w0 = voice.left_w0 + 0.1

        # rotate the left arm outward
        elif command == 'left red': 
            voice.left_w0 = voice.left_w0 - 0.4

        elif command == 'left orange': 
            voice.left_w0 = voice.left_w0 - 0.1

        # rotate the left arm back to zero
        elif command == 'left blue': 
            voice.left_w0 = 0.0
                       
        else:
            print("No valid command")
            return

if __name__=="__main__":

    try:
        rospy.init_node('voice', anonymous = True)
        voice = Voice()
        print("Going to start the while loop")
        
        
        while not rospy.is_shutdown():
            
            voice.right_w0_pub.publish(voice.right_w0)
            voice.right_w1_pub.publish(voice.right_w1)
            voice.right_w2_pub.publish(voice.right_w2)

            voice.left_w0_pub.publish(voice.left_w0)
            voice.left_w1_pub.publish(voice.left_w1)
            voice.left_w2_pub.publish(voice.left_w2)
            voice.pub.publish(voice.message)
            voice.r.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")

