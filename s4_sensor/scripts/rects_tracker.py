#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import Rect
from jsk_recognition_msgs.msg import RectArray
from s4_msgs.msg import TrackedRect
from s4_msgs.msg import TrackedRectArray
import numpy as np

class rectsTracker:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.rects_pub = rospy.Publisher('tracking_rects', TrackedRectArray, queue_size=1)
        self.rects_sub = rospy.Subscriber('rects', RectArray, self.rects_callback, queue_size=1)
        self.tracking_elements=[]
        self.last_id=0

        #params
        self.debug = rospy.get_param("~debug", False)
        self.size_weight = rospy.get_param("~size_weight", 1)
        self.max_distance = rospy.get_param("~max_distance", 100)
        self.min_width = rospy.get_param("~min_width", 10)
        self.min_height = rospy.get_param("~min_height", 10)
        self.min_age = rospy.get_param("~min_age", 5)

        if(self.debug):
            self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        
    def rects_callback(self, rects_msg):
        if(not self.tracking_elements):
            self.reset_rects(rects_msg.rects)
        else:
            self.update_rects(rects_msg.rects)

        tr_array_msg=TrackedRectArray()
        tr_array_msg.header=rects_msg.header
        for tr in self.tracking_elements:
            if(tr.width>self.min_width and tr.height>self.min_height):
                if(tr.age>self.min_age):
                    tr_array_msg.rects.append(tr)

        self.rects_pub.publish(tr_array_msg)

    def reset_rects(self, rects):
        for rect_msg in rects:
            rect_element=TrackedRect()
            rect_element.x=rect_msg.x
            rect_element.y=rect_msg.y
            rect_element.width=rect_msg.width
            rect_element.height=rect_msg.height
            rect_element.id=self.last_id
            self.last_id+=1
            rect_element.age=0
            self.tracking_elements.append(rect_element)
        
    def update_rects(self, rects):
        if(not rects):
            self.tracking_elements=[]
        else:
            relation=np.zeros((len(self.tracking_elements),len(rects)))
            inheritance=np.zeros((len(rects)))
            inheritance[:]=-1
            
            new_tracking_elements=[]
            relation[:,:]=float('inf')
            for i in range(len(self.tracking_elements)):
                for j in range(len(rects)):
                    relation[i,j]=self.getDistance(self.tracking_elements[i], rects[j], self.size_weight)

            for step in range(min(len(self.tracking_elements), len(rects))):
                minarg=np.unravel_index(np.argmin(relation), relation.shape)
                if self.max_distance<relation[minarg]:
                    break
                inheritance[minarg[1]]=minarg[0]
                for i in range(len(self.tracking_elements)):
                    relation[i, minarg[1]]=float('inf')
                for j in range(len(rects)):
                    relation[minarg[0], j]=float('inf')

            for j in range(len(inheritance)):
                rect_element=TrackedRect()
                rect_element.x=rects[j].x
                rect_element.y=rects[j].y
                rect_element.width =rects[j].width
                rect_element.height=rects[j].height
                if(inheritance[j]>=0):
                    rect_element.id =self.tracking_elements[int(inheritance[j])].id
                    rect_element.age=self.tracking_elements[int(inheritance[j])].age+1
                else:
                    rect_element.id =self.last_id
                    self.last_id+=1
                    rect_element.age=0
                new_tracking_elements.append(rect_element)                

            self.tracking_elements=new_tracking_elements

    def getDistance(self, elem1, elem2, size_weight):
        dx=(elem1.x+elem1.width/2) -(elem2.x+elem2.width/2)
        dy=(elem1.y+elem1.height/2)-(elem2.y+elem2.height/2)
        dw=elem1.width-elem2.width
        dh=elem1.height-elem2.height
        return np.sqrt(dx**2+dy**2)+np.sqrt(dw**2+dh**2)*size_weight

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)        
        output_image = self.process_image(input_image, True)
        
        if self.debug:
            cv2.imshow("overlap rects", output_image)
            cv2.waitKey(1)

    def process_image(self, image, debug=False):
        if(self.tracking_elements is not None):
            for rect_element in self.tracking_elements:
                if(rect_element.width>self.min_width and rect_element.height>self.min_height):
                    if(rect_element.age>self.min_age):
                        cv2.rectangle(image, (rect_element.x, rect_element.y), (rect_element.x+rect_element.width, rect_element.y+rect_element.height), (255, 0, 0), thickness=2)
                        text="id:"+str(rect_element.id)+",age:"+str(rect_element.age)
                        cv2.putText(image,text,(rect_element.x, rect_element.y),cv2.FONT_HERSHEY_PLAIN, 2,(255,0,0), thickness=2)
                    else:
                        cv2.rectangle(image, (rect_element.x, rect_element.y), (rect_element.x+rect_element.width, rect_element.y+rect_element.height), (0, 255, 0), thickness=2)
        return image

    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    rectsTracker()
    rospy.spin()