#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
import numpy as np

class hsvFilter:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.rects_pub = rospy.Publisher('raw_rects', RectArray, queue_size=1)
        #params
        self.debug = rospy.get_param("~debug", False)
        self.reduction = rospy.get_param("~reduction", 2)
        self.h_min = rospy.get_param("~h_min", 0)
        self.h_max = rospy.get_param("~h_max", 180)
        self.s_min = rospy.get_param("~s_min", 0)
        self.s_max = rospy.get_param("~s_max", 255)
        self.v_min = rospy.get_param("~v_min", 0)
        self.v_max = rospy.get_param("~v_max", 255)
        
    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)
        
        rects_msg = self.process_image(input_image, self.debug)
        rects_msg.header=ros_image.header
        self.rects_pub.publish(rects_msg)

    def process_image(self, image, debug=False):
        # resize
        orgHeight, orgWidth = image.shape[:2]
        image = cv2.resize(image, (orgWidth / self.reduction, orgHeight / self.reduction))

        # hsv filter
        if(self.h_min < self.h_max):
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([self.h_min, self.s_min, self.v_min])
            upper_red = np.array([self.h_max, self.s_max, self.v_max])
            mask = cv2.inRange(hsv, lower_red, upper_red)

        else:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([self.h_min, self.s_min, self.v_min])
            upper_red1 = np.array([180,        self.s_max, self.v_max])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            lower_red2 = np.array([0,          self.s_min, self.v_min])
            upper_red2 = np.array([self.h_max, self.s_max, self.v_max])
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask=mask1+mask2

        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("hsv filter", display)
            
        # morphology processing
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)
        
        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("morphology processing", display)   
        
        # make contour
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if debug:
            display = np.zeros(mask.shape, dtype=np.uint8)
            for c in contours:
                for elem in c:
                    display[elem[0,1],elem[0,0]]=255
            cv2.imshow("make contours", display)   
        
        # make region
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(rect)
            
        rects_msg=RectArray()
        for rect in rects:
            cv2.rectangle(image, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0), thickness=1)
            rect_msg=Rect()
            rect_msg.x =rect[0]*self.reduction
            rect_msg.y =rect[1]*self.reduction
            rect_msg.width  =rect[2]*self.reduction
            rect_msg.height =rect[3]*self.reduction
            rects_msg.rects.append(rect_msg)

        if debug:
            cv2.imshow("make region", image)
        
        if debug:
            cv2.waitKey(1)
           
        return rects_msg

    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    hsvFilter()
    rospy.spin()