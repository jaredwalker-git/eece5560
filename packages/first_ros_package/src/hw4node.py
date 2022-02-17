#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled

class HW4Node:
    def __init__(self):
    	
        rospy.Subscriber('/mystery/output2', UnitsLabelled, self.callback)
        self.publisher = rospy.Publisher("unitconv", UnitsLabelled, queue_size = 10)
        
    def callback(self, msg):
    	
        if rospy.has_param("Units"):
            param = rospy.get_param("Units")
            if param == 'feet':
                meters = msg.value
                msg.units = "feet"
                msg.value = meters * 3.2808
                self.publisher.publish(msg)
            elif param == 'smoots':
                meters = msg.value
                msg.units = "smoots"
                msg.value = meters * 1.7018
                self.publisher.publish(msg)
            else:
                self.publisher.publish(msg)
            
if __name__ == "__main__":
    rospy.init_node('hw4node')
    node = HW4Node()
    rospy.spin()
            

