#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from s4_msgs.msg import TrackedObjectArray, TrackedInfo

class hsvFilter:
    def __init__(self):
        self.node_name = "image_overlay"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("view_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.objects_sub = rospy.Subscriber("input_objects", TrackedObjectArray, self.objects_callback, queue_size=1)
        self.last_objects = TrackedObjectArray()
        self.focus_sub = rospy.Subscriber("focus", TrackedInfo, self.focus_callback, queue_size=1)
        self.last_focus = TrackedInfo()

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        image = np.array(frame, dtype=np.uint8)

        #objects
        for object in self.last_objects.objects:
            color = (0, 255, 0)
            if self.last_focus.category == object.info.category and self.last_focus.id == object.info.id:
                color = (255, 100, 100)
            r = object.rect
            cv2.rectangle(image, (r.x, r.y), (r.x + r.width, r.y + r.height), color, thickness=3)
            text1 = object.info.category
            text2 = "id:" + str(object.info.id)
            cv2.putText(image, text1, (r.x, r.y - 25), cv2.FONT_HERSHEY_PLAIN, 1.5,(0, 255, 0), thickness=2)
            cv2.putText(image, text2, (r.x, r.y -  5), cv2.FONT_HERSHEY_PLAIN, 1.5,(0, 255, 0), thickness=2)

        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(msg)


    def objects_callback(self, objects_msg):
        self.last_objects = objects_msg

    def focus_callback(self, focus_msg):
        self.last_focus = focus_msg

    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    hsvFilter()
    rospy.spin()