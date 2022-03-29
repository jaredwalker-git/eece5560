#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosAgent:
    def __init__(self):
        self.image_receiver = rospy.Subscriber('/image', Image, self.image_process)
        self.split_img_pub = rospy.Publisher('/image_cropped', Image, queue_size = 10)
        self.cvbridge = CvBridge()
        self.whitepub = rospy.Publisher('/image_white', Image, queue_size = 10)
        self.yellowpub = rospy.Publisher('/image_yellow', Image, queue_size = 10)
        
    def image_process(self, img): #images 640 X 480
        cvimg = self.cvbridge.imgmsg_to_cv2(img, "bgr8")
        split_img = cvimg[239:479, :]
        hsvimg = cv2.cvtColor(split_img, cv2.COLOR_BGR2HSV)
        
        white_filteredhsv = cv2.inRange(hsvimg, (0, 0, 20), (180, 20, 255))
        yellow_filteredhsv = cv2.inRange(hsvimg, (29, 60, 20), (31, 255, 255))
        
        #white_filteredbgr = cv2.cvtColor(white_filteredhsv, cv2.COLOR_HSV2BGR)
        #yellow_filteredbgr = cv2.cvtColor(white_filteredhsv, cv2.COLOR_HSV2BGR)
        
        white_imgmsg = self.cvbridge.cv2_to_imgmsg(white_filteredhsv, "mono8")
        yellow_imgmsg = self.cvbridge.cv2_to_imgmsg(yellow_filteredhsv, "mono8")
        split_imgmsg = self.cvbridge.cv2_to_imgmsg(split_img, "bgr8")
        
        self.split_img_pub.publish(split_imgmsg)
        self.whitepub.publish(white_imgmsg)
        self.yellowpub.publish(yellow_imgmsg)
            

if __name__ == '__main__':
    rospy.init_node('hw7node', anonymous = True)
    agent = RosAgent()
    rospy.spin()    
