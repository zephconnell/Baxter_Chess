#! /usr/bin/env python

import rospy
import baxter_interface
from sensor_msgs.msg import JointState


def callback(data):
    
    test = True
    names = []
    positions = []
    names = data.name
    positions = data.position
    for j in names:
        if j == 'l_gripper_l_finger_joint':
            test = False
            return
        elif j == 'r_gripper_l_finger_joint':
            test = False
            return
        else:
            print('\n'*1)
            
    
    if test:
        left_s0 = names[4]
        left_s0_value = positions[4]
        left_s1 = names[5]
        left_s1_value = positions[5]
        left_e0 = names[2]
        left_e0_value = positions[2]
        left_e1 = names[3]
        left_e1_value = positions[3]
        left_w0 = names[6]
        left_w0_value = positions[6]
        left_w1 = names[7]
        left_w1_value = positions[7]
        left_w2 = names[8]
        left_w2_value = positions[8]

        right_s0 = names[11]
        right_s0_value = positions[11]
        right_s1 = names[12]
        right_s1_value = positions[12]
        right_e0 = names[9]
        right_e0_value = positions[9]
        right_e1 = names[10]
        right_e1_value = positions[10]
        right_w0 = names[13]
        right_w0_value = positions[13]
        right_w1 = names[14]
        right_w1_value = positions[14]
        right_w2 = names[15]
        right_w2_value = positions[15]
        print("***************************************************")
        print("The value of %s is %.2f." %(left_s0, left_s0_value)) 
        print("The value of %s is %.2f." %(left_s1, left_s1_value))
        print("The value of %s is %.2f." %(left_e0, left_e0_value))
        print("The value of %s is %.2f." %(left_e1, left_e1_value))
        print("The value of %s is %.2f." %(left_w0, left_w0_value))
        print("The value of %s is %.2f." %(left_w1, left_w1_value))
        print("The value of %s is %.2f." %(left_w2, left_w2_value))
        print('\n' *2)
        print("The value of %s is %.2f." %(right_s0, right_s0_value))
        print("The value of %s is %.2f." %(right_s1, right_s1_value))
        print("The value of %s is %.2f." %(right_e0, right_e0_value))
        print("The value of %s is %.2f." %(right_e1, right_e1_value))
        print("The value of %s is %.2f." %(right_w0, right_w0_value))
        print("The value of %s is %.2f." %(right_w1, right_w1_value))
        print("The value of %s is %.2f." %(right_w2, right_w2_value))
        print("*****************************************************")
            

def listener():
    rospy.init_node('subscriber_to_robot_joint_states')
    rospy.Subscriber('/robot/joint_states', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


