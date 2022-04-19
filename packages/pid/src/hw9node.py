#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from pid import pid_controller

class ros_agent:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/error", Float32, self.call_pid)
        self.control_pub = rospy.Publisher("/control_input", Float32, queue_size = 10)
        self.pid_controller = pid_controller(rospy.get_time())
        
    def call_pid(self, error):
        control = controller.control_update(error, rospy.get_time())
        self.control_pub.publish(control)

if __name__ == '__main__':
    rospy.init_node('hw9node')
    controller = pid_controller(rospy.get_time())
    agent = ros_agent()
    rospy.set_param("/controller_ready", "true")
    rospy.spin()
