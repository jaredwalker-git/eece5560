#!/usr/bin/env python3

import rospy
import actionlib
import example_action_server.msg 
from example_service.srv import Fibonacci


if __name__ == '__main__':
    order = 15
    rospy.init_node('hw10node')
        
    #service
    rospy.loginfo("waiting for service")
    rospy.wait_for_service('/calc_fibonacci')
    rospy.loginfo("Service found")
    try:
        fibsrv = rospy.ServiceProxy('/calc_fibonacci', Fibonacci)
        rospy.loginfo("Service Proxied")
        t1 = rospy.get_time()
        srv_seq = fibsrv(order)
        rospy.loginfo("Fib finished")
        rospy.logwarn("Service return for order " + str(order) + ": " + str(srv_seq.sequence))
        rospy.logwarn("Service time elapsed: " + str(rospy.get_time()-t1))
    except:
        rospy.loginfo("Service Failed")
        
    #action
    rospy.loginfo("beginning actionclient")
    t2 = rospy.get_time()
    act_client = actionlib.SimpleActionClient('/fibonacci', example_action_server.msg.FibonacciAction)
    rospy.loginfo("waiting for server")
    act_client.wait_for_server()
    rospy.loginfo("server found")
    t3 = rospy.get_time()
    act_client.send_goal(example_action_server.msg.FibonacciGoal(order))
    rospy.loginfo("goal sent")
    act_client.wait_for_result()
    rospy.logwarn("Goal return time:" + str(rospy.get_time() - t3))
    rospy.loginfo("result recieved")
    rospy.logwarn("Service return for order " + str(order) + ": " + str(act_client.get_result()))
    rospy.logwarn("Action time elapsed: " + str(rospy.get_time()-t2))
    
