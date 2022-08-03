import open3d as o3d
import numpy as np
from traversability_layer import ros_o3d_bridge

def filter_z(pcl_data, base=0, window = 0.5):
    np_array = np.asarray(pcl_data)
    o3d_pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(np_array)) #`voxel_down_sample(voxel_size=0.05)` if it's too sluggish

    ## From Open3D to numpy
    np_points = np.asarray(o3d_pcd.points)
    np_filter = []
    for point in np_points:
        if (base - window) < (point[2] - base) < (base + window):
            np_filter.append(False)
        else:
            np_filter.append(True)

    return np_points[np_filter]
