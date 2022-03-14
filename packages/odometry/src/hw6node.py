#!/usr/bin/env python3

import numpy as np
import rospy
from odometry_hw.msg import Pose2D, DistWheel

class traveltoCoords:

    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.findpose)
        self.publisher = rospy.Publisher('/pose', Pose2D, queue_size = 10)
        self.wheeldist = 0.05 #dist from center robot to wheel
        self.currentx = 0
        self.currenty = 0
        self.currenttheta = 0

    def findpose(self, msg):
        totaldist = (msg.dist_wheel_right + msg.dist_wheel_left)/2
        anglechange = (msg.dist_wheel_right - msg.dist_wheel_left)/(2 * self.wheeldist)
        changex = totaldist * np.cos(self.currenttheta + anglechange/2)
        changey = totaldist * np.sin(self.currenttheta + anglechange/2)
        self.currentx = self.currentx + changex
        self.currenty = self.currenty + changey
        self.currenttheta = self.currenttheta + anglechange
        pubmsg.x = self.currentx
        pubmsg.y = self.currenty
        pubmsg.theta = self.currenttheta
        self.publisher.publish(pubmsg)

if __name__ == "__main__":
    rospy.init_node('hw6node')
    pubmsg = Pose2D()
    poseFinder = traveltoCoords()
    rospy.spin()
	
	
		
		
