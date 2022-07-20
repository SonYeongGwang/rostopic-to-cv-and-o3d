#! /usr/bin/python
import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rgb_image = []
depth_image= []
def rgb_image_callback(data):
    global rgb_image
    cv_bridge1 = CvBridge()
    try:
        rgb_image = cv_bridge1.imgmsg_to_cv2(data)
        rgb_image = np.array(rgb_image)
    except CvBridgeError as e:
        print (e)

def depth_image_callback(data):
    global depth_image
    cv_bridge2 = CvBridge()
    try:
        depth_image = cv_bridge2.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_image = np.array(depth_image)
    except CvBridgeError as e:
        print (e)
    
rospy.init_node('image_getter', anonymous=True)
rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
rospy.Subscriber('/camera/color/image_raw', Image, rgb_image_callback)
rgb_pub = rospy.Publisher('rgb_image', Image, queue_size=10)
depth_pub = rospy.Publisher('depth_image', Image, queue_size=10)

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    if len(rgb_image) != 0:
        cv2.imshow('rgb', rgb_image)
        cv2.imshow('depth', depth_image)
        cv2.waitKey(1)
        
        rgbMsg = Image()
        rgbMsg.header.stamp = rospy.Time.now()
        rgbMsg.height = 480
        rgbMsg.width = 640
        rgbMsg.encoding = "bgr8"
        rgbMsg.is_bigendian = False
        rgbMsg.step = 3 * 640
        rgbMsg.data = rgb_image.tobytes()

        depthMsg = Image()
        depthMsg.header.stamp = rospy.Time.now()
        depthMsg.height = 480
        depthMsg.width = 640
        depthMsg.encoding = "32FC1"
        depthMsg.is_bigendian = False
        depthMsg.step = 640
        depthMsg.data = depth_image.tobytes()

        rgb_pub.publish(rgbMsg)
        depth_pub.publish(depthMsg)
        rate.sleep()
