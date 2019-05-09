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

    
if __name__ == '__main__':
    R = tf.transformations.euler_matrix(0, 0, math.pi/2)
    T = tf.transformations.translation_matrix((1, 0, 0))
    ex = np.mat([1,0,0,1]).T
    # print(T*ex)
    # print(R*T*ex)
    print(R)
    print(R*ex)
    print(T*(R*ex))
    #print(tf.transformations.decompose_matrix(R))