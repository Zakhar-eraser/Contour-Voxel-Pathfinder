#import laspy as lp
import numpy as np
import open3d as o3d

input_path = 'C:/Users/Центр БЛА/projects/'
dataname = 'voxel_mai.ply'
voxel_size = 1

pcd = o3d.io.read_point_cloud(input_path+dataname)
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

def pick_points():
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()

def intersections_count():
    

#o3d.visualization.draw_geometries([voxel_grid])
#octree = o3d.geometry.Octree(max_depth=10)
#octree.create_from_voxel_grid(voxel_grid)
#o3d.visualization.draw_geometries([octree])

o3d.visualization.draw_geometries([voxel_grid])