#!/usr/bin/env python3

import rospy
import cv2
import os.path
from os import path
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosAgent:
    def __init__(self):
        self.cropped_sub = rospy.Subscriber('/image_cropped', Image, self.edge_detect_cropped)
        self.white_sub = rospy.Subscriber('/image_white', Image, self.whiteimg_store)
        self.yellow_sub = rospy.Subscriber('/image_yellow', Image, self.yellowimg_store)
        self.edge_publisher = rospy.Publisher('/image_edges', Image, queue_size = 10)
        self.white_line_pub = rospy.Publisher('/image_lines_white', Image, queue_size = 10)
        self.yellow_line_pub = rospy.Publisher('/image_lines_yellow', Image, queue_size = 10)
        self.hough_line_pub = rospy.Publisher('/image_lines_all', Image, queue_size = 10)
        self.bridge = CvBridge()
        self.image_counter = 0
        self.edge_recieved = False
        self.yellow_recieved = False
        self.white_recieved = False
        self.cropped_img = []
        self.edge_imgs = []
        self.white_imgs = []
        self.yellow_imgs = []
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
    def edge_detect_cropped(self, img):
        self.cropped_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        edge_image = cv2.Canny(self.cropped_img, 175, 220) 
        self.edge_imgs = cv2.dilate(edge_image, self.kernel)
        edgeimgmsg = self.bridge.cv2_to_imgmsg(self.edge_imgs, "mono8")
        self.edge_publisher.publish(edgeimgmsg)
        self.edge_recieved = True
        
        if (self.white_recieved) and (self.yellow_recieved) and (self.edge_recieved):
            
            def create_houghimg(img_overlay):
                houghlines = []
                img_overlay = np.array(img_overlay)
                houghlines = cv2.HoughLinesP(img_overlay, 1, 3.1415/180, 50)
                white_line_image = self.output_lines(self.cropped_img, houghlines)
                line_imgmsg = self.bridge.cv2_to_imgmsg(white_line_image, "bgr8")
                return line_imgmsg
                
            img_overlay_white = []
            img_overlay_white = np.array(img_overlay_white)
            img_overlay_white = cv2.bitwise_and(np.array(self.white_imgs), np.array(self.edge_imgs)).astype('uint8')
            img_overlay_yellow = []
            img_overlay_yellow = np.array(img_overlay_yellow)
            img_overlay_yellow = cv2.bitwise_and(np.array(self.yellow_imgs), np.array(self.edge_imgs)).astype('uint8')
            line_msg_white = create_houghimg(img_overlay_white)
            line_msg_yellow = create_houghimg(img_overlay_yellow)
                        
            img_overlay_both = cv2.bitwise_or(np.array(img_overlay_white), np.array(img_overlay_yellow)).astype('uint8')
            line_imgmsg = create_houghimg(img_overlay_both)
            
            self.yellow_line_pub.publish(line_msg_yellow)
            self.white_line_pub.publish(line_msg_white)
            self.hough_line_pub.publish(line_imgmsg)
            
                    
    def whiteimg_store(self, img):
        self.white_imgs = self.bridge.imgmsg_to_cv2(img, "mono8")
        self.white_recieved = True
        
    def yellowimg_store(self, img):    
        self.yellow_imgs = self.bridge.imgmsg_to_cv2(img, "mono8")
        self.yellow_recieved = True
        
if __name__ == "__main__":
    rospy.init_node('hw8node')
    agent = RosAgent()
    rospy.spin()
        
        
