#! /usr/bin/env python
"""
simple script that subscribes to
the simulated robot's topic
/robot/joint_states.
It allows you to track the robots current
joint angles that are being published on the
topic.
Note that this will not work on the real robot
It is for the simulated only. 

"""
import rospy
import baxter_interface
from sensor_msgs.msg import JointState
count = 0
def callback(data):
    global count
    if count % 5 == 0:
        left_s0 = data.name[5]
        left_s0_value = data.position[5]
        left_s1 = data.name[6]
        left_s1_value = data.position[6]
        left_e0 = data.name[3]
        left_e0_value = data.position[3]
        left_e1 = data.name[4]
        left_e1_value = data.position[4]
        left_w0 = data.name[7]
        left_w0_value = data.position[7]
        left_w1 = data.name[8]
        left_w1_value = data.position[8]
        left_w2 = data.name[9]
        left_w2_value = data.position[9]

        right_s0 = data.name[14]
        right_s0_value = data.position[14]
        right_s1 = data.name[15]
        right_s1_value = data.position[15]
        right_e0 = data.name[12]
        right_e0_value = data.position[12]
        right_e1 = data.name[13]
        right_e1_value = data.position[13]
        right_w0 = data.name[16]
        right_w0_value = data.position[16]
        right_w1 = data.name[17]
        right_w1_value = data.position[17]
        right_w2 = data.name[18]
        right_w2_value = data.position[18]
   
        #print("The value of %s is %.2f." %(left_s0, left_s0_value)) 
        #print("The value of %s is %.2f." %(left_s1, left_s1_value))
        #print("The value of %s is %.2f." %(left_e0, left_e0_value))
        #print("The value of %s is %.2f." %(left_e1, left_e1_value))
        #print("The value of %s is %.2f." %(left_w0, left_w0_value))
        #print("The value of %s is %.2f." %(left_w1, left_w1_value))
        #print("The value of %s is %.2f." %(left_w2, left_w2_value))
        #print('\n' *1)
        print("The value of %s is %.2f." %(right_s0, right_s0_value))
        print("The value of %s is %.2f." %(right_s1, right_s1_value))
        print("The value of %s is %.2f." %(right_e0, right_e0_value))
        print("The value of %s is %.2f." %(right_e1, right_e1_value))
        print("The value of %s is %.2f." %(right_w0, right_w0_value))
        print("The value of %s is %.2f." %(right_w1, right_w1_value))
        print("The value of %s is %.2f." %(right_w2, right_w2_value))
        
        print('\n' * 4)
    count += 1
        

def listener():
    rospy.init_node('subscriber_to_robot_joint_states')
    rospy.Subscriber('/robot/joint_states', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


