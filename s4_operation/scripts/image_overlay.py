#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo, LaserScan, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from s4_msgs.msg import TrackedObjectArray, TrackedInfo
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import math
import tf
import image_geometry

class hsvFilter:
    def __init__(self):
        self.node_name = "image_overlay"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.tf_prefix = rospy.get_param('~tf_prefix')
        rpy_offset_name = rospy.get_param('~rpy_offset')
        self.offset_roll = rospy.get_param(rpy_offset_name + "/roll")
        self.offset_pitch = rospy.get_param(rpy_offset_name + "/pitch")
        self.offset_yaw = rospy.get_param(rpy_offset_name + "/yaw")

        self.image_pub = rospy.Publisher("view_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.objects_sub = rospy.Subscriber("input_objects", TrackedObjectArray, self.objects_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_callback, queue_size=1)
        self.last_laser = None
        self.last_objects = TrackedObjectArray()
        self.focus_sub = rospy.Subscriber("focus", TrackedInfo, self.focus_callback, queue_size=1)
        self.last_focus = TrackedInfo()
        self.voltage_sub = rospy.Subscriber("voltage", Float32, self.voltage_callback, queue_size=1)
        self.last_voltage = 0.0
        self.point_sub = rospy.Subscriber("point", Point, self.point_callback, queue_size=1)
        self.last_point = Point()
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=1)
        self.last_info = CameraInfo

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

        #laser
        lp = lg.LaserProjection()
        if self.last_laser is not None:
            center_u = 640
            center_v = 210
            scale = 200 # m -> px

            box_pu = 150
            box_nu = 150
            box_pv = 50
            box_nv = 200

            (trans, quat) = self.listener.lookupTransform(self.tf_prefix + '/base_link', self.tf_prefix + '/sensor0/front_laser_link', rospy.Time(0))
            rpy = tf.transformations.euler_from_quaternion(quat)
            offset_x = trans[0]
            offset_y = trans[1]
            rotate_z = rpy[2]
            
            color = (50, 50, 250)
            limit_nu = center_u - box_nu
            limit_nv = center_v - box_nv
            limit_pu = center_u + box_pu
            limit_pv = center_v + box_pv

            cv2.rectangle(image, (limit_nu, limit_nv), (limit_pu, limit_pv), color, thickness=3)
            cv2.circle(image,(int(center_u),int(center_v)), int(scale * 0.12), (0,0,255), 3)

            pc2_msg = lp.projectLaser(self.last_laser)
            point_list = pc2.read_points_list(pc2_msg)
            for point0 in point_list:
                point_x = math.cos(rotate_z) * point0.x - math.sin(rotate_z) * point0.y + offset_x
                point_y = math.sin(rotate_z) * point0.x + math.cos(rotate_z) * point0.y + offset_y
                point_u = int(center_u - scale * point_y)
                point_v = int(center_v - scale * point_x)
                if limit_nu <= point_u <= limit_pu and limit_nv <= point_v <= limit_pv:
                    cv2.circle(image,(point_u, point_v), 3, (0,0,255), -1)

        # voltage
        text_v = "V: " + "{:.1f}".format(self.last_voltage)
        cv2.putText(image, text_v, (10,50), cv2.FONT_HERSHEY_PLAIN, 3.0,(0, 0, 255), thickness=3)

        # point
        if self.last_point.x > 0.1:
            (trans,rot) = self.listener.lookupTransform(self.tf_prefix + '/sensor0/head_camera_link', self.tf_prefix + '/gun0/standard', rospy.Time(0))
            R = tf.transformations.quaternion_matrix(rot)
            T = tf.transformations.translation_matrix(trans)
            CR = tf.transformations.euler_matrix(-self.offset_roll, -self.offset_pitch, -self.offset_yaw)
            
            px = np.array([self.last_point.x, self.last_point.y, self.last_point.z]).T
            px1 = px / np.linalg.norm(px) * 0.5
            px2 = px / np.linalg.norm(px) * 1.5
            
            px1 = np.mat([px1[0], px1[1], px1[2], 1]).T
            out1 =CR*(T*(R*px1))
            px2 = np.mat([px2[0], px2[1], px2[2], 1]).T
            out2 =CR*(T*(R*px2))

            pp = image_geometry.PinholeCameraModel()
            pp.fromCameraInfo(self.last_info)
            uv1 = pp.project3dToPixel(out1[0:3])
            uv2 = pp.project3dToPixel(out2[0:3])

            cv2.circle(image, (int(uv1[0]), int(uv1[1])), 30, (0,0,255), 2)
            cv2.circle(image, (int(uv2[0]), int(uv2[1])), 20, (0,0,255), 2)


        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(msg)


    def objects_callback(self, objects_msg):
        self.last_objects = objects_msg

    def focus_callback(self, focus_msg):
        self.last_focus = focus_msg

    def laser_callback(self, laser_msg):
        self.last_laser = laser_msg

    def voltage_callback(self, float_msg):
        self.last_voltage = float_msg.data

    def point_callback(self, point_msg):
        self.last_point = point_msg

    def info_callback(self, info_msg):
        self.last_info = info_msg

    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    hsvFilter()
    rospy.spin()