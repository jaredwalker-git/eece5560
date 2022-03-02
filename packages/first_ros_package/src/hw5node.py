#!/usr/bin/env python3

import numpy as np
import rospy
from duckietown_msgs.msg import Vector2D


class Locator:
    def __init__(self):
        #initialize ROS stuff
        self.robot_coord = rospy.Publisher('/robot_coord', Vector2D)
        self.world_coord = rospy.Publisher('/world_coord', Vector2D)
        rospy.Subscriber('/input_coord', Vector2D, self.coordChng)
    
        #initialize class variables for coordinate translations\
        self.scalars = np.array([0, 0, 1])
        self.sensorRobot = np.matrix([[-1],[0]]) #sensor relative robot 
        self.sensorRot = np.matrix([[np.cos(180), -np.sin(180)],[np.sin(180), np.cos(180)]])
        self.robotWorld = np.matrix([[3],[2]]) #robot relative world 
        self.robotRot = np.matrix([[np.cos(135), -np.sin(135)],[np.sin(135),np.cos(135)]])
        self.transSensor = np.append(self.sensorRot, self.sensorRobot, axis = 1) #trans sensor to robot
        self.transSensor = np.append(self.transSensor, self.scalars.reshape(1,3), axis = 0)
        self.transRobot = np.append(self.robotRot, self.robotWorld, axis = 1)
        self.transRobot = np.append(self.transRobot, self.scalars.reshape(1,3), axis = 0)



    def coordChng(self, msg):
    
        pointVector = np.matrix([[msg.x],[msg.y]])
        pointVector = np.append(pointVector, np.ones([1,1]), axis = 0)
        robotCoords = np.matmul(self.transSensor, pointVector)
        worldCoords = np.matmul(self.transRobot, robotCoords)
        xrCoords = robotCoords[0, 0] #indexing x variable for robot coords
        yrCoords = robotCoords[1, 0] #indexing y variable for robot coords
        xwCoords = worldCoords[0, 0] #indexing x variable for world coords
        ywCoords = worldCoords[1, 0] #indexing y variable for world coords
        self.robot_coord.publish(xrCoords, yrCoords)
        self.world_coord.publish(xwCoords, ywCoords)
                                 
        
if __name__ == "__main__":
    rospy.init_node('hw5node')
    location = Locator()
    rospy.spin()
    
