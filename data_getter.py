import cv2
import rospy

import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Image, PointCloud2

rgb_image = []
depth_image = []
pcd = o3d.geometry.PointCloud()

def rgb_image_callback(data):
    global rgb_image
    rgb_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

def depth_image_callback(data):
    global depth_image
    depth_image = np.frombuffer(data.data, dtype=np.float32).reshape(data.height, data.width)

def point_cloud_callback(data):

    global pcd

    # Get cloud data from ros point cloud
    field_names=[field.name for field in data.fields]
    cloud_data = list(pc2.read_points(data, skip_nans=True, field_names = field_names))

    if len(cloud_data) == 0:
        print('No cloud data!')
        return None

    xyz = [(x,y,z) for x,y,z, _ in cloud_data ] # get xyz
    pcd.points = o3d.utility.Vector3dVector(np.array(xyz))

rospy.init_node('Ros_Node', anonymous=True)

rospy.Subscriber('/rgb_image', Image, rgb_image_callback)
rospy.Subscriber('/depth_image', Image, depth_image_callback)
rospy.Subscriber('/camera/depth/points', PointCloud2, point_cloud_callback)

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    ###########################################
    #             user application            #
    ###########################################
    pass