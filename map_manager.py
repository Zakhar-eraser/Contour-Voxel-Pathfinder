import numpy as np
#import laspy as lp
import open3d as o3d
import os

maps_dir = "maps/"

def create_project(path, voxel_size):
    filename = os.path.splitext(os.path.basename(path))[0]
    if not os.path.exists(maps_dir): os.mkdir(maps_dir)

    project_dir = maps_dir + filename + '/'
    file = project_dir + filename + ".ply"
    if not os.path.exists(project_dir):
        os.mkdir(project_dir)
       # lp_pcd = lp.read(path)
        #pcd = o3d.geometry.PointCloud()
        #pcd.points = o3d.utility.Vector3dVector(np.vstack((lp_pcd.X, lp_pcd.Y, lp_pcd.Z)).transpose())
        #pcd.colors = o3d.utility.Vector3dVector(np.vstack((lp_pcd.red, lp_pcd.green, lp_pcd.blue)).transpose()/65535)
        pcd = o3d.io.read_point_cloud(path)
        pcd = pcd.voxel_down_sample(voxel_size)
        o3d.io.write_point_cloud(file, pcd)
    return file
