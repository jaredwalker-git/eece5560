#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class HW3Node:
    def __init__(self):
    	
        rospy.Subscriber('/mystery/output2', UnitsLabelled, self.callback)
        self.publisher = rospy.Publisher("meterstofeet", UnitsLabelled)
        
    def callback(self, msg):
        meters = msg.value
        msg.units = "feet"
        msg.value = meters * 3.2808
        self.publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node('hw3node')
    node = HW3Node()
    rospy.spin()
        
    
