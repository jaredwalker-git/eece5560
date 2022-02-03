#!/usr/bin/env python3

import numpy
from numpy import random
import time
import rospy
from std_msgs.msg import Float32

class RandNode:
	def __init__(self):
		self.publisher = rospy.Publisher('/mystery/input', Float32)
		
	def get_number(self):
		number = random.randint(0,11)
		rospy.loginfo(number)
		self.publisher.publish(number)
		
if __name__ == '__main__':
	
	node = RandNode()
	rospy.init_node('randnum')

	while not rospy.is_shutdown():
		node.get_number()
		time.sleep(1)
		

