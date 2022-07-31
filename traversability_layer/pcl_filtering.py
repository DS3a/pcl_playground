import open3d as o3d
import numpy as np

def filter_z(pcl_data, window = 0.05):
    np_array = np.asarray(pcl_data)
    o3d_pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np_array))

    ## From Open3D to numpy
    ##np_points = np.asarray(pcd.points)
    print(np_array.shape)
