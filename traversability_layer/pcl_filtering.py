import open3d as o3d
import numpy as np

def filter_z(window=0.2, pcl_data: o3d.geometry.PointCloud):
    np_array = np.asarray(pcl_data.points)
    print(np_array.shape)
