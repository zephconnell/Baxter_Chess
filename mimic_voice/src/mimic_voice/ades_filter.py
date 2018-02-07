#!/usr/bin/env python
# see license.txt
"""
This is the adaptive double exponential smoothing filter which a filtering
option from the Microsoft White Paper entitled:
"Skeletal Joint Smoothing White Paper"
Here is the link to the paper:
https://msdn.microsoft.com/en-us/library/jj131429.aspx

The idea to use this filter came from the NeuroScience and Robotics laboratory
at Northwestern University. They have site here:
https://github.com/NxRLab/nxr_baxter.  They used this type of filter for their
Baxter demo.

It is tried here because it takes into account how quickly the joint angles
are changing.  When there is little change between positions, there is more
aggressive filtering.  When the joints are moving quickly, different parameters 
are used resulting in better responsiveness to the change (lower latency).

The Microsoft white paper describes each filter and pros/cons

Try out the different filters which are included in this package:
1) Simple moving average
2) Double moving average
3) Adaptive double exponential smoothing

See which works best for you. 
You can also change parameters....those below are from the example
above except for the self.v_low which is different.  

"""

import numpy as np
import rospy



class ADESfilter:
    def __init__(self):
        
        #filter parameter
        self.gamma = 0.01
        self.a_low=0.01
        self.a_high=0.35
        self.v_low=0.008
        self.v_high=0.01
        self.bn_l = [0.0, 0.0, 0.0, 0.0]
        self.bn_r = [0.0,0.0,0.0,0.0]
        self.prev_bn_l = [0.0, 0.0, 0.0, 0.0]
        self.prev_val_l = [0.0, 0.0, 0.0, 0.0]
        self.prev_bn_r = [0.0, 0.0, 0.0, 0.0]
        self.prev_val_r = [0.0, 0.0, 0.0, 0.0]

    def ades_filter(self, limb_name,ades, **angles):
       
        angle_s0 = angles["s0"]
  
        angle_s1 = angles["s1"]

        angle_e0 = angles["e0"]
        angle_e1 = angles["e1"] 
        

        filtered_angle = [0.0,0.0,0.0,0.0]
        
        if limb_name == "left":
            left = []

            left.append(angle_s0)
            left.append(angle_s1)
            left.append(angle_e0)
            left.append(angle_e1)
            print("The value of x is: ", left)
            index = 0
            for x in left:
                print ("The value of x is : ", x)
                val = ades.update_filter_l(left[index],index)
                filtered_angle[index] = val
                index = index + 1
            angle_s0 = filtered_angle[0]
            angle_s1 = filtered_angle[1]
            angle_e0 = filtered_angle[2]
            angle_e1 = filtered_angle[3]
            print("The value of filtered_angle is: ", filtered_angle)
            return {'s0':angle_s0, 's1':angle_s1, 'e0':angle_e0, 'e1':angle_e1}

        else:
            right = []

            right.append(angle_s0)
            right.append(angle_s1)
            right.append(angle_e0)
            right.append(angle_e1)
            print("The value of x is: ", right)
            index = 0
            for x in right:
                print ("The value of x is : ", x)
                val = ades.update_filter_r(right[index],index)
                filtered_angle[index] = val
                index = index + 1
            angle_s0 = filtered_angle[0]
            angle_s1 = filtered_angle[1]
            angle_e0 = filtered_angle[2]
            angle_e1 = filtered_angle[3]
            print("The value of filtered_angle is: ", filtered_angle)
            return {'s0':angle_s0, 's1':angle_s1, 'e0':angle_e0, 'e1':angle_e1}

    def update_filter_l(self,val,index):
        
        vn = np.abs(val - self.prev_val_l[index])
        if vn < self.v_low:
            self.falpha = self.a_low
        elif self.v_low <= vn <= self.v_high:
            self.falpha = self.a_high + ((vn-self.v_high)/(self.v_low-self.v_high))*\
              (self.a_low-self.a_high)
        elif vn > self.v_high:
            self.falpha = self.a_high
        else:
            self.falpha = (self.a_high+self.a_low)/2.0

        val = self.falpha*val + (1-self.falpha)*(self.prev_val_l[index]+self.bn_l[index])
        self.bn_l[index] = self.gamma*(val-self.prev_val_l[index]) + (1-self.gamma)*self.prev_bn_l[index]
        self.prev_bn_l[index] = self.bn_l[index]
        self.prev_val_l[index] = val
        print ("The value of val is: ", val)
        return val
    def update_filter_r(self,val,index):
        vn = np.abs(val - self.prev_val_r[index])
        if vn < self.v_low:
            self.falpha = self.a_low
        elif self.v_low <= vn <= self.v_high:
            self.falpha = self.a_high + ((vn-self.v_high)/(self.v_low-self.v_high))*\
              (self.a_low-self.a_high)
        elif vn > self.v_high:
            self.falpha = self.a_high
        else:
            self.falpha = (self.a_high+self.a_low)/2.0

        val = self.falpha*val + (1-self.falpha)*(self.prev_val_r[index]+self.bn_r[index])
        self.bn_r[index] = self.gamma*(val-self.prev_val_r[index]) + (1-self.gamma)*self.prev_bn_r[index]
        self.prev_bn_r[index] = self.bn_r[index]
        self.prev_val_r[index] = val
        print ("The value of val is: ", val)
        return val

if __name__ == '__main__':
    
    test = {'s0': 1, 's1': 2, 'e0': 3,'e1':4}
    test1 = {'s0': 1.5, 's1': 2.5, 'e0': 3,'e1':4}
    test2 = {'s0': 1.5, 's1': 2.5, 'e0': 3,'e1':4}
    rospy.init_node("test")
    a = ADESfilter()
    a.ades_filter('left', **test)
    a.ades_filter('left',**test1)
    a.ades_filter('left',**test2)


    

