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

        self.rate = 5
        self.r = rospy.Rate(self.rate)

        self.message = 'sayonara'
        
        # A flag to determine whether or not voice control is paused
        self.paused = False

	# A flag to determine if the given command is the pickup index or placement index
	self.movementdefined = False

	# variables for the starting and ending positions
	self.departure = -1
	self.destination = -1

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
		return

	index = -1
        
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
	elif command == 'twenty one':  
            index = 21;
	elif command == 'twenty two':  
            index = 22;
	elif command == 'twenty three':  
            index = 23;
	elif command == 'twenty four':  
            index = 24;
	elif command == 'twenty five':  
            index = 25;
	elif command == 'twenty six':  
            index = 26;
	elif command == 'twenty seven':  
            index = 27;
	elif command == 'twenty eight':  
            index = 28;
	elif command == 'twenty nine':  
            index = 29;
	elif command == 'thirty':  
            index = 30;
	elif command == 'thirty one':  
            index = 31;
	elif command == 'thirty two':  
            index = 32;
	elif command == 'thirty three':  
            index = 33;
	elif command == 'thirty four':  
            index = 34;
	elif command == 'thirty five':  
            index = 35;
	elif command == 'thirty six':  
            index = 36;
	elif command == 'thirty seven':  
            index = 37;
	elif command == 'thirty eight':  
            index = 38;
	elif command == 'thirty nine':  
            index = 39;
	elif command == 'fourty':  
            index = 40;
	elif command == 'fourty one':  
            index = 41;
	elif command == 'fourty two':  
            index = 42;
	elif command == 'fourty three':  
            index = 43;
	elif command == 'fourty four':  
            index = 44;
	elif command == 'fourty five':  
            index = 45;
	elif command == 'fourty six':  
            index = 46;
	elif command == 'fourty seven':  
            index = 47;
	elif command == 'fourty eight':  
            index = 48;
	elif command == 'fourty nine':  
            index = 49;
	elif command == 'fifty':  
            index = 50;
	elif command == 'fifty one':  
            index = 51;
	elif command == 'fifty two':  
            index = 52;
	elif command == 'fifty three':  
            index = 53;
	elif command == 'fifty four':  
            index = 54;
	elif command == 'fifty five':  
            index = 55;
	elif command == 'fifty six':  
            index = 56;
	elif command == 'fifty seven':  
            index = 57;
	elif command == 'fifty eight':  
            index = 58;
	elif command == 'fifty nine':  
            index = 59;
	elif command == 'sixty':  
            index = 60;
	elif command == 'sixty one':  
            index = 61;
	elif command == 'sixty two':  
            index = 62;
	elif command == 'sixty three':  
            index = 63;
	elif command == 'reset':
	    self.departure = -1
	    self.destination = -1
	    self.movementdefined = False
        else:
            print("No valid command")
            return
	
	if self.departure == -1 and index >= 0 and index <= 63:
		self.departure = index
	elif self.destination == -1 and index >= 0 and index <= 63:
		self.destination = index
		self.movementdefined = True

	if self.movementdefined:
		self.message = 'placeholder'
		

if __name__=="__main__":

    try:
        rospy.init_node('voicecommands', anonymous = True)
        voice = Voice()
        print("Going to start the while loop")
        
        
        while not rospy.is_shutdown():
	    if voice.movementdefined:
                voice.pub.publish(voice.message)
		print("Move piece at postion %d to postion %d." % (voice.departure, voice.destination))
		voice.departure = -1
		voice.destination = -1
		voice.movementdefined = False
            voice.r.sleep()
	    print("Ready for verbal input")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice navigation terminated.")

