#! /usr/bin/env python
#Please see the license.txt file for the open source licenses for these two references
"""
References:
"Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart.  Chapter 19 is devoted to making the robot talk using 
   pyttsx. Pyttsx is text to speech. Code was used from the book.  The code for the book is available at (i) and the documentation for
   pyttsx is available at (ii)
   (i)  https://github.com/osrf/rosbook
   (ii) https://pyttsx.readthedocs.io/en/latest/

"""
import rospy

import actionlib
from mimic_voice.msg import TalkAction, TalkGoal, TalkResult

class TalkMimic():

    def talk_client(self):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = "I will get your position from the skeleton tracker and then follow your arm movements."
    	client.send_goal(goal)
    	client.wait_for_result()
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))


    

if __name__ =="__main__":
    try:
        rospy.init_node('speaker_client5', anonymous = True)
        talk1 = Talk()
        talk1.talk_client()
        #talk1=talk_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
