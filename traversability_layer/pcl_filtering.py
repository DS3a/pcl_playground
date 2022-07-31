import open3d as o3d
import numpy as np

def filter_z(pcl_data, window = 0.05):
    np_array = np.asarray(pcl_data.points)
    print(np_array.shape)
