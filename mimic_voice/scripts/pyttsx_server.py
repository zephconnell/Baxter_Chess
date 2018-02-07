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
import threading, time, pyttsx
import actionlib
from mimic_voice.msg import TalkAction, TalkGoal, TalkResult

class TalkNode():

    def __init__(self, node_name, action_name):
        rospy.init_node(node_name, anonymous = True)
        self.server = actionlib.SimpleActionServer(action_name, TalkAction,
          self.do_talk, False)
        self.engine = pyttsx.init()
        rate = self.engine.getProperty('rate')
        voices = self.engine.getProperty('voices')
        self.engine_thread = threading.Thread(target=self.loop)
        self.engine_thread.start()
        self.engine.setProperty('volume', rospy.get_param('~volume', 1.0))
        self.engine.setProperty('rate', rate - 90)
        self.engine.setProperty('voice', voices[16].id)
        self.preempt = rospy.get_param('~preempt', False)
        self.server.start()

    def loop(self):
        self.engine.startLoop(False)
        while not rospy.is_shutdown():
            self.engine.iterate()
            time.sleep(0.1)
        self.engine.endLoop()
    
    def do_talk(self, goal):
        self.engine.say(goal.sentence)
        while self.engine.isBusy():
            if self.preempt and self.server.is_preempt_requested():
                self.engine.stop()
                while self.engine.isBusy():
                    time.sleep(0.1)
                self.server.set_preempted(TalkResult(), "Talk preempted")
                return
            time.sleep(0.1)
        self.server.set_succeeded(TalkResult(), "Talk completed successfully")

talker = TalkNode('speaker', 'speak')
rospy.spin()
