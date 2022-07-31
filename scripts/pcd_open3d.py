#!/usr/bin/python3


import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2

# import util
import open3d as o3d
import numpy as np

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.o3d_pcd = o3d.geometry.PointCloud()


        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/velodyne_points',                      # topic
            self.listener_callback,      # Function to call
            10                          # QoS
        )

                
    def listener_callback(self, msg):

        # pcl_data = util.convert_pcl(msg)
        pcl_data = msg # od somthing to convert to pcl
        tmp_pcl = pcl_data
        pc_p = np.asarray(pcl_data.points)
        pc_c = np.asarray(pcl_data.points)
        pc_c[:,0] = 0
        pc_c3d = o3d.Vector3dVector(pc_c)
        tmp_pcl.colors = pc_c3d
        return tmp_pcl
        # util.publish_pointcloud(tmp_pcl, msg)



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

