#!/usr/bin/env python3

import rospy
import cv2
import os.path
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosAgent:
    def __init__(self):
        self.cropped_sub = rospy.Subscriber('/image_cropped', Image, self.image_process)
        self.white_sub = rospy.Subscriber('/image_white', Image, self.whiteimg_store)
        self.yellow_sub = rospy.Subscriber('/image_yellow', Image)
        self.edge_publisher = rospy.Publisher('/image_edges', Image, queue_size = 10)
        self.image_line_pub = rospy.Publisher('/image_lines_all', Image, queue_size = 10)
        self.bridge = CvBridge()
        self.img_counter = 0
        self.cropped_buffer = [0] * 5
        self.edge_buffer = [0] * 5
        self.white_buffer = [0] * 5
        self.yellow_buffer = [0] * 5
        
        
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
    def image_process(self, img):
        cvImg = self.bridge.imgmsg_to_cv2(img, "bgr8")
        edge_image = cv2.Canny(cvImg, 100, 200)
        edgeimgmsg = self.bridge.cv2_to_imgmsg(edge_image, "mono8")
        self.edge_publisher.publish(edgeimgmsg)
        
        t = rospy.get_time() #time when img recieved
        #while not rospy.is_shutdown() and rospy.get_time() - t < 5: #wait 5s
        self.cropped_buffer[self.img_counter] = cvImg #store imgs
        self.edge_buffer[self.img_counter] = edge_image
        #rospy.Rate(1).sleep
        
        self.img_counter =+ 1 #index number of imgs taken
            
            
        if self.img_counter == 5:
            img_overlay = [0] * 5 
            houghlines = [0] * 5
            white_line_image = [0] * 5
            yellow_line_image = [0] * 5
            line_imgmsg = [0] * 5
            
            for i in range(len(self.white_buffer)):
                img_overlay[i] = cv2.bitwise_and(self.white_buffer[i], self.edge_buffer[i])
                houghlines[i] = cv2.HoughLinesP(img_overlay[i], 1, 3.1415/180, 50)
                white_line_image[i] = self.output_lines(self.cropped_buffer[i], houghlines[i])
                line_imgmsg[i] = self.bridge.cv2_to_imgmsg(white_line_image[i], "bgr8")
                 
            while not rospy.is_shutdown(): #have run while ros is running    
                k = 0
                for k in range(5):
                    t2 = rospy.get_time() #time when pub started 
                    k =+ 1
                    while not rospy.is_shutdown() and rospy.get_time() - t2 < 5: #wait 5s
                         self.image_line_pub.publish(line_imgmsg[k])
                         rospy.Rate(5).sleep()
            
        
    def whiteimg_store(self, img):
        cvImg = self.bridge.imgmsg_to_cv2(img, "mono8")
        for i in range(5):
            t = rospy.get_time() #time when img recieved
            while not rospy.is_shutdown() and rospy.get_time() - t < 5.05:
                 self.white_buffer.append(cvImg)    
            return
            
if __name__ == "__main__":
    rospy.init_node('hw8node')
    agent = RosAgent()
    rospy.spin()
        
        
