#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

def hello():
    #Now we enable Baxter's motors using the Baxter API
    # First we create an instance of the RobotEnable class from the baxter_interface
    # Then we can use the "menu" from the RobotEnable class and enable Baxter's motors
    robot = baxter_interface.RobotEnable(CHECK_VERSION)
    robot.enable()    
    
    #Let's create a instance of both the right and left arms so we
    # can control both arms.
    right_limb = baxter_interface.Limb('right')
    left_limb = baxter_interface.Limb('left')

    # Let's set both arms to neutral position
    right_limb.move_to_neutral()
    left_limb.move_to_neutral()

    #get the angles for the right arm
    angles = right_limb.joint_angles()
 
    # print the angles
    print('\n')
    print ("This shows Baxter's right arm angles when they are in neutral position: {0}.".format(angles))
    print('\n' *2)

    # reassign new joint angles to the right arm
    angles = {'right_s0': 0.0, 'right_s1': 0.0,'right_e0': 0.0,'right_e1': 0.0,
               'right_w0': 0.0,'right_w1': 0.0,'right_w2': 0.0}
 
    print ("We reset Baxter's right arm angles to zero as this now shows: {0}.".format(angles))
    print('\n' *2)

    # move the right arm to the new position
    right_limb.move_to_joint_positions(angles)

    #Now create the two arm positions needed for baxter to wave
    
    wave_1 = {'right_s0': -0.459, 'right_s1': -0.202,'right_e0': 1.807,'right_e1': 1.714,
               'right_w0': -0.906,'right_w1': -1.545,'right_w2': -0.276}

 
    wave_2 = {'right_s0': -0.395, 'right_s1': -0.202,'right_e0': 1.831,'right_e1': 1.981,
               'right_w0': -1.979,'right_w1': -1.100,'right_w2': -0.448}

    """ 
    wave_1 = {'right_s0': 0.72, 'right_s1': -0.40,'right_e0': -0.12,'right_e1': 0.38,
               'right_w0': 0.00,'right_w1': 0.27,'right_w2': 1.38}

 
    wave_2 = {'right_s0': -0.12, 'right_s1': -0.32,'right_e0': -0.89,'right_e1': -0.96,
               'right_w0': 0.75,'right_w1': 1.41,'right_w2': 1.11}
    """
    for x in range(3):
        right_limb.move_to_joint_positions(wave_1)
        right_limb.move_to_joint_positions(wave_2)
        
    
    #move the robot's arms to neutral position
    right_limb.move_to_neutral()
    left_limb.move_to_neutral()

    #disable the robot's motors
    robot.disable()
    

if __name__ == '__main__':
    # we need to have a node for the program
    rospy.init_node('Hello_Baxter')

    #call the hello method
    hello()
    print("All done.  Have a nice day")

    
    
