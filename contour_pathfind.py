import numpy as np
import open3d as o3d

input_path = 'monu1.ply'
voxel_size = 1

def pick_points(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()

#def intersections_count():
    
pcd = o3d.io.read_point_cloud(input_path)
print("Yellow sphere is the start point, Blue sphere is the target point")
mission_points_ids = list(set(pick_points(pcd)))
assert (len(mission_points_ids) != 2), "It must be exactly 2 picked points"
target_point = np.asarray(pcd.points)[mission_points_ids[0]]
start_point = np.asarray(pcd.points)[mission_points_ids[1]]
pcd = pcd.voxel_down_sample(voxel_size)
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
