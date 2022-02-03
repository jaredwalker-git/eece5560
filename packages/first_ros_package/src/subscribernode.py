#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callbackfunc(msg):
	rospy.loginfo("mystery/output1 published %f" %msg.data)

if __name__ == '__main__':
	rospy.init_node('subnode', anonymous = True)
	rospy.Subscriber('/mystery/output1', Float32, callbackfunc)
	rospy.spin()
	
	
