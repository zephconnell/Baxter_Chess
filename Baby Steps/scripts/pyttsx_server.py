#! /usr/bin/env python
#Please see the license.txt file for the open source licenses for these two references
"""
References:
"Programming Robots with ROS" by Morgan Quigley, Brian Gerkey, and William Smart.  Chapter 19 is devoted to making the robot talk using 
   pyttsx. Pyttsx is text to speech. Code was used from the book.  The code for the book is available at (i) and the documentation for
   pyttsx is available at (ii)
   (i)  https://github.com/osrf/rosbook
   (ii) https://pyttsx.readthedocs.io/en/latest/

Reference for implementing a simple action server is from the ROS Wiki Tutorial Web site:
http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29


"""
import rospy
import threading, time, pyttsx
import actionlib
from baby_steps.msg import TalkAction, TalkGoal, TalkResult

class TalkNode():

    def __init__(self, node_name, action_name):
        rospy.init_node(node_name, anonymous = True)
        #first create a simple action server. Arguments are a name, an action, 
        # a call_back, and set auto_start to False
        self.server = actionlib.SimpleActionServer(action_name, TalkAction,
          self.do_talk, False)
        #get a reference to an engine reference from the pyttsx library
        self.engine = pyttsx.init()
        """
        In the next section, we declare a variable used to set the properties for
        the robot voice we will be setting the rate which is integer speech rate in
        words per minute.
        voices is a list of the available "voices" in pyttsx.  We will be using
        id 16.
        We also start a thread to run the speech engine loop
        """
        rate = self.engine.getProperty('rate')
        voices = self.engine.getProperty('voices')
        self.engine_thread = threading.Thread(target=self.loop)
        self.engine_thread.start()
        self.engine.setProperty('volume', rospy.get_param('~volume', 1.0))
        self.engine.setProperty('rate', rate - 90)
        self.engine.setProperty('voice', voices[16].id)
        self.preempt = rospy.get_param('~preempt', False)
        #start the action server
        self.server.start()

    def loop(self):
        self.engine.startLoop(False)
        while not rospy.is_shutdown():
            #when using an external event loop, engine.iterate must
            # be used within the external loop per pyttsx docs
            self.engine.iterate()
            time.sleep(0.1)
        self.engine.endLoop()
    
    def do_talk(self, goal):
        #when the server receives a goal to speak, it passes it to the speech engine
        # we created. 
        self.engine.say(goal.sentence)
        while self.engine.isBusy():
            # action servers allow the action to be preempted. If a 
            # request is received then the action would be stopped by using
            # the speech libraries stop function for the engine we created.
            if self.preempt and self.server.is_preempt_requested():
                self.engine.stop()
                while self.engine.isBusy():
                    time.sleep(0.1)
                self.server.set_preempted(TalkResult(), "Talk preempted")
                return
            time.sleep(0.1)
        self.server.set_succeeded(TalkResult(), "Talk completed successfully")

#create an instance of the TalkNode class
talker = TalkNode('speaker', 'speak')
#keep the server spinning
rospy.spin()
