#!/usr/bin/env python


import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
from std_msgs.msg import (
      String,
      Float64,
)

import rospkg

)
"""
KEY FOR THE VOICE COMMANDS

"""

class Voice:
    def __init__(self):
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print("Enabling robot... ")
        rs.enable()

        self.paused = False
        self.rate = 5
        self.r = rospy.Rate(self.rate)

        self.message = 'sayonara'
        
        # A flag to determine whether or not voice control is paused
        self.paused = False

	# A flag to determine if the given command is the pickup index or placement index
	self.placement = False

        #publishing the verbal command to the command topic
        self.pub = rospy.Publisher('/command', String, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)      
        
         
    def speech_callback(self, msg):
        
        # Get the motion command from the recognized phrase
        command = msg.data
        
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
        
	if command == 'shutdown'
		#do good shutdown thing

        # The list of if-then statements should be fairly
        # self-explanatory

	index = -1
	pickup = -1
	placement = -1
        
        # get index of piece to move or index to move to                     
        if command == 'zero':  
            index = 0;
	elif command == 'one':  
            index = 1;
	elif command == 'two':  
            index = 2;
	elif command == 'three':  
            index = 3;
	elif command == 'four':  
            index = 4;
	elif command == 'five':  
            index = 5;
	elif command == 'six':  
            index = 6;
	elif command == 'seven':  
            index = 7;
	elif command == 'eight':  
            index = 8;
	elif command == 'nine':  
            index = 9;
	elif command == 'ten':  
            index = 10;
	elif command == 'eleven':  
            index = 11;
	elif command == 'twelve':  
            index = 12;
	elif command == 'thirteen':  
            index = 13;
	elif command == 'fourteen':  
            index = 14;
	elif command == 'fifteen':  
            index = 15;
	elif command == 'sixteen':  
            index = 16;
	elif command == 'seventeen':  
            index = 17;
	elif command == 'eighteen':  
            index = 18;
	elif command == 'nineteen':  
            index = 19;
	elif command == 'twenty':  
            index = 20;
	elif command == 'twentyone':  
            index = 21;
	elif command == 'twentytwo':  
            index = 22;
	elif command == 'twentythree':  
            index = 23;
	elif command == 'twentyfour':  
            index = 24;
	elif command == 'twentyfive':  
            index = 25;
	elif command == 'twentysix':  
            index = 26;
	elif command == 'twentyseven':  
            index = 27;
	elif command == 'twentyeight':  
            index = 28;
	elif command == 'twentynine':  
            index = 29;
	elif command == 'thirty':  
            index = 30;
	elif command == 'thirtyone':  
            index = 31;
	elif command == 'thirtytwo':  
            index = 32;
	elif command == 'thirtythree':  
            index = 33;
	elif command == 'thirtyfour':  
            index = 34;
	elif command == 'thirtyfive':  
            index = 35;
	elif command == 'thirtysix':  
            index = 36;
	elif command == 'thirtyseven':  
            index = 37;
	elif command == 'thirtyeight':  
            index = 38;
	elif command == 'thirtynine':  
            index = 39;
	elif command == 'fourty':  
            index = 40;
	elif command == 'fourtyone':  
            index = 41;
	elif command == 'fourtytwo':  
            index = 42;
	elif command == 'fourtythree':  
            index = 43;
	elif command == 'fourtyfour':  
            index = 44;
	elif command == 'fourtyfive':  
            index = 45;
	elif command == 'fourtysix':  
            index = 46;
	elif command == 'fourtyseven':  
            index = 47;
	elif command == 'fourtyeight':  
            index = 48;
	elif command == 'fourtynine':  
            index = 49;
	elif command == 'fifty':  
            index = 50;
	elif command == 'fiftyone':  
            index = 51;
	elif command == 'fiftytwo':  
            index = 52;
	elif command == 'fiftythree':  
            index = 53;
	elif command == 'fiftyfour':  
            index = 54;
	elif command == 'fiftyfive':  
            index = 55;
	elif command == 'fiftysix':  
            index = 56;
	elif command == 'fiftyseven':  
            index = 57;
	elif command == 'fiftyeight':  
            index = 58;
	elif command == 'fiftynine':  
            index = 59;
	elif command == 'sixty':  
            index = 60;
	elif command == 'sixtyone':  
            index = 61;
	elif command == 'sixtytwo':  
            index = 62;
	elif command == 'sixtythree':  
            index = 63;
        else:
            print("No valid command")
            return

if __name__=="__main__":

    try:
        rospy.init_node('voicecommands', anonymous = True)
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

