#!/usr/bin/env python3

import numpy as np
import rospy
from odometry_hw.msg import Pose2D

class currentCoords:

    def __init__(self):
        rospy.Subscriber('/duckiekong/right_wheel_encoder_node/tick', WheelsCmdStamped, self.storepose_right)
        rospy.Subscriber('/duckiekong/left_wheel_encoder_node/tick', WheelsCmdStamped, self.findpose_both)
        self.publisher = rospy.Publisher('/pose', Pose2D, queue_size = 10)
        self.wheeldist = 0.05 #dist from center robot to wheel
        self.left_wheel_tick = 0 
        self.right_wheel_tick = 0
        self.right_wheel_tick_last = 0
        self.left_wheel_tick_last = 0
        self.update_flag = False
        self.currentx = 0
        self.currenty = 0
        self.currenttheta = 0

    
    def storepose_right(self, right_ticks):
        self.right_wheel_tick_last = self.right_wheel_tick
        self.right_wheel_tick = right_ticks.data
        #689.5,677 / 669.5, 673.50 -> avg 677 ticks/meter
        self.update_flag = True
        
    def findpose_both(self, left_ticks):
        self.left_wheel_tick_last = self.left_wheel_tick
        self.left_wheel_tick = left_ticks.data
        
        if update_flag = True:
            #ticks to meters
            right_tick_change = self.right_wheel_tick - self.right_wheel_tick_last
            left_tick_change = self.left_wheel_tick - self.left_wheel_tick_last
            totaldist = (right_tick_change + left_tick_change)/2
        
            anglechange = (right_tick_change + left_tick_change)/(2 * self.wheeldist)
            changex = totaldist * np.cos(self.currenttheta + anglechange/2)
            changey = totaldist * np.sin(self.currenttheta + anglechange/2)
            self.currentx = self.currentx + changex
            self.currenty = self.currenty + changey
            self.currenttheta = self.currenttheta + anglechange
            pubmsg.x = self.currentx
            pubmsg.y = self.currenty
            pubmsg.theta = self.currenttheta
            self.publisher.publish(pubmsg)
            self.update_flag = False

if __name__ == "__main__":
    rospy.init_node('lab2prt2')
    pubmsg = Pose2D()
    poseFinder = findpose()
    rospy.spin()
