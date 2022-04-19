#!/usr/bin/env python3
from std_msgs.msg import Float32
import rospy
import numpy as np

class pid_controller:
    def __init__(self, time):
    

        self.error_0 = 0
        self.error_1 = 0
        self.time_0 = time
        self.time_1 = 0
        if rospy.has_param("/k_p"):
            self.k_p = rospy.get_param("/k_p")         
        if rospy.has_param("/k_i"):
            self.k_i = rospy.get_param("/k_i")
        if rospy.has_param("/k_d"):
            self.k_d = rospy.get_param("/k_d")
        
        
    def control_update(self, error, current_time):
        self.error_1 = self.error_0
        self.error_0 = error.data
        self.time_1 = self.time_0
        self.time_0 = current_time
        time_change = self.time_0 - self.time_1

        p = self.k_p * self.error_0
        i = self.k_i * self.error_0 * time_change #estimate of integral
        d = self.k_d * ((self.error_0 - self.error_1) / time_change) #estimate of rate of change
        control = p + i + d
        return control
 
    
