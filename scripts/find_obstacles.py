#!/usr/bin/python3

import sys
import os

import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import pcd_open3d
import sensor_msgs.msg._point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2


from traversability_layer import pcl_filtering
import numpy as np
import open3d as o3d

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        #self.vis = o3d.visualization.Visualizer()
        #self.vis.create_window()
        #self.o3d_pcd = o3d.geometry.PointCloud()


        # Set up a subscription to the 'pcd' topic with a callback to the
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/velodyne_points',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )


    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from
        # the ROS1 package.
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py
        # open3d_cloud = o3d.PointCloud()
        pcd_as_numpy_array = np.array(list(pcd_open3d.read_points(msg)))
        # field_names=[field.name for field in msg.fields]
        # cloud_data = list(pc2.read_points(msg, skip_nans=True, field_names = field_names))
        # xyz = [(x,y,z) for x,y,z in cloud_data ]
        # The rest here is for visualization.
        # self.vis.remove_geometry(self.o3d_pcd)
        self.op = o3d.utility.Vector3dVector(pcd_as_numpy_array)
        # self.o3d_pcd = o3d.geometry.PointCloud(self.op)
        # open3d_cloud.points = o3d.Vector3dVector(pcd_as_numpy_array)
        #o3d.visualization.draw_geometries(pcd_as_numpy_array)
        #self.vis.add_geometry(self.o3d_pcd)
        #self.vis.poll_events()
        #self.vis.update_renderer()
    # self.pointcloud2_to_array(pass)

## The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later:
# https://github.com/ros2/common_interfaces




def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
