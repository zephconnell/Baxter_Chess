#! /usr/bin/env python
#Please see the license.txt file for the open source licenses for these two references
"""
References:
"Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart.  Chapter 19 is devoted to making the robot talk using 
   pyttsx. Pyttsx is text to speech. Code was used from the book.  The code for the book is available at (i) and the documentation for
   pyttsx is available at (ii)
   (i)  https://github.com/osrf/rosbook
   (ii) https://pyttsx.readthedocs.io/en/latest/

Reference to create a simple action client:
http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29

"""
import rospy

import actionlib
from baby_steps.msg import TalkAction, TalkGoal, TalkResult

class Talk():

    def say_piece(self):  
    	#rospy.init_node('speaker_client')
        #create the client by passing in as arguments the name and type of action
        #in this case, we are using the TalkAction.action that we created
    	client = actionlib.SimpleActionClient('speak', TalkAction)
        #we wait until the action server has started and is accepting goals
    	client.wait_for_server()
        #TalkGoal was created when we wrote the Talk.action file and then compiled it
        # In Talk.action, we define the goal as string sentence. Thus the data type
        # is a string and the goal is sentence
    	goal = TalkGoal()
    	goal.sentence = "Say the piece you would like to move, or say a functional operation."
        #send the goal to the action server. In this case, the goal is to say our sentence.
    	client.send_goal(goal)
        #wait for the server to give us our results.
    	client.wait_for_result()
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))

    def start_init(self):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = "Press Enter to start initialization."
    	client.send_goal(goal)
    	client.wait_for_result()
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))

    def begin_voice(self):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = "Press Enter to begin voice command operation."
    	client.send_goal(goal)
    	client.wait_for_result()
        #use simple_action_client function to get state i.e. pending,
        # preempting, or active and print this result
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))

    def check_move(self):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = "Did I successfully make the move requested? Reply 'Success' or 'Failure'."
    	client.send_goal(goal)
    	client.wait_for_result()
        #use simple_action_client function to get state i.e. pending,
        # preempting, or active and print this result
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))

    def invalid_move(self):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = "Requested move was invalid. Say the piece you would like to move, or say a functional operation."
    	client.send_goal(goal)
    	client.wait_for_result()
        #use simple_action_client function to get state i.e. pending,
        # preempting, or active and print this result
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))
    
    def say_location(self, piece):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = piece + " has been chosen as the piece to move. Give the location to which you would like to move it."
    	client.send_goal(goal)
    	client.wait_for_result()
        #use simple_action_client function to get state i.e. pending,
        # preempting, or active and print this result
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))

    def location_chosen(self, location):  
    	#rospy.init_node('speaker_client')
    	client = actionlib.SimpleActionClient('speak', TalkAction)
    	client.wait_for_server()
    	goal = TalkGoal()
    	goal.sentence = piece + " has been chosen as the piece to move. Give the location to which you would like to move it."
    	client.send_goal(goal)
    	client.wait_for_result()
        #use simple_action_client function to get state i.e. pending,
        # preempting, or active and print this result
    	print('[Result] State: %d'%(client.get_state()))
    	print('[Result] Status: %s'%(client.get_goal_status_text()))
    

if __name__ =="__main__":
    try:
        rospy.init_node('speaker_client', anonymous = True)
        talk1 = Talk()
        talk1.talk_client()
        #talk1=talk_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


