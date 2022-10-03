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

def get_bounding_idx(voxels):
    min_idx = np.int32[3, 1]
    min_idx[0] = min(voxels, key=lambda voxel: voxel.grid_index[0])
    min_idx[1] = min(voxels, key=lambda voxel: voxel.grid_index[1])
    min_idx[2] = min(voxels, key=lambda voxel: voxel.grid_index[2])
    max_idx = np.int32[3, 1]
    max_idx[0] = max(voxels, key=lambda voxel: voxel.grid_index[0])
    max_idx[1] = max(voxels, key=lambda voxel: voxel.grid_index[1])
    max_idx[2] = max(voxels, key=lambda voxel: voxel.grid_index[2])
    return [min_idx, max_idx]

def get_occupancy_grid(boundaries, voxels):
    

#def intersections_count():
def main():
    pcd = o3d.io.read_point_cloud(input_path)
    print("Yellow sphere is the start point, Blue sphere is the target point")
    mission_points_ids = list(set(pick_points(pcd)))
    assert (len(mission_points_ids) != 2), "It must be exactly 2 picked points"

    target_point = np.asarray(pcd.points)[mission_points_ids[0]]
    start_point = np.asarray(pcd.points)[mission_points_ids[1]]

    pcd = pcd.voxel_down_sample(voxel_size)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    target_voxel = voxel_grid.get_voxel(target_point)
    start_voxel = voxel_grid.get_voxel(start_point)
    print(target_voxel.grid_index)
    print(start_voxel.grid_index)

    voxels = voxel_grid.get_voxels()
    bounding_idx = get_bounding_idx(voxels)
    print(bounding_idx[0][1])



if __name__ == '__main__':
    main()
