#!/usr/bin/python3


import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField

from traversability_layer import pcl_filtering, ros_o3d_bridge
import open3d as o3d
import numpy as np

class PCDListener(Node):

    def __init__(self, debug=False):
        super().__init__('pcd_subsriber_node')

        ## This is for visualization of the received point cloud.
        self.debug = debug
        if self.debug:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.o3d_pcd = o3d.geometry.PointCloud()
        # Set up a subscription to the 'pcd' topic with a callback to the
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            '/velodyne_points',                      # topic
            #'/camera/aligned_depth_to_color/color/points',
            self.listener_callback,      # Function to call
            10                          # QoS
        )
        self.points = None
        self.time = None
        self.travsersible_points_publisher = self.create_publisher(
            PointCloud2,
            '/traversible_points',
            10
        )
        self.non_travsersible_points_publisher = self.create_publisher(
            PointCloud2,
            '/non_traversible_points',
            10
        )

    def listener_callback(self, msg):

        self.points = msg
        self.time = msg.header.stamp
        pcd_as_numpy_array = np.array(list(ros_o3d_bridge.read_points(msg)))[:, :3]
        print(pcd_as_numpy_array.shape)

        filtered_points = pcl_filtering.filter_z(pcd_as_numpy_array, 0, 0.5)
        send_msg = ros_o3d_bridge.np_to_point_cloud(filtered_points, '/velodyne', self.time)
        self.non_travsersible_points_publisher.publish(send_msg)

        # The rest here is for visualization.
        if self.debug:
            self.vis.remove_geometry(self.o3d_pcd)
            self.o3d_pcd = o3d.geometry.PointCloud(
                            o3d.utility.Vector3dVector(pcd_as_numpy_array)).voxel_down_sample(voxel_size=1.09)
            self.vis.add_geometry(self.o3d_pcd)
            self.vis.poll_events()
            self.vis.update_renderer()

def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener(debug=False)
    rclpy.spin(pcd_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
