import numpy as np
import open3d as o3d
import A_star_pathfinder

input_path = 'monu2.ply'
voxel_size = 1.0
max_observe_dist = 20.0

def pick_points(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    return vis.get_picked_points()

def get_max_bounding_idx(pcd):
    max_bound = pcd.get_max_bound()
    min_bound = pcd.get_min_bound()
    max_idx = (max_bound - min_bound) / voxel_size
    max_idx = max_idx.astype('int32')

def get_occupancy_grid(boundary, voxels):
    grid = np.zeros(boundary[0], boundary[1], boundary[2])
    for vox in voxels:
        grid[vox.grid_index[0], vox.grid_index[1], vox.grid_index[2]] = 1
    return grid

def points_sqr_len(p1, p2):
    dp = p1 - p2
    return dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]

def check_intersection(start_voxel, target_voxel, occupancy_grid):
    occupancy_grid[target_voxel[0], target_voxel[1], target_voxel[2]] = 0
    intersect = 0
    cur_voxel = start_voxel
    x1 = start_voxel[0]
    y1 = start_voxel[1]
    z1 = start_voxel[2]
    dX = target_voxel[0] - start_voxel[0]
    dY = target_voxel[1] - start_voxel[1]
    dZ = target_voxel[2] - start_voxel[2]
    vox_step = voxel_size / 2
    B = np.full(shape=3, fill_value=vox_step, dtype=np.float32)
    if dX < 0: B[0] = -B[0]
    if dY < 0: B[1] = -B[1]
    if dZ < 0: B[2] = -B[2]
    sX = 1 if dX > 0 else -1
    sY = 1 if dY > 0 else -1
    sZ = 1 if dZ > 0 else -1
    dXdY = dX / dY
    dXdZ = dX / dZ
    dYdX = dY / dX
    dYdZ = dY / dZ
    dZdX = dZ / dX
    dZdY = dZ / dY
    x_by_y = lambda y: dXdY * (y - y1)
    x_by_z = lambda z: dXdZ * (z - z1)
    y_by_x = lambda x: dYdX * (x - x1)
    y_by_z = lambda z: dYdZ * (z - z1)
    z_by_x = lambda x: dZdX * (x - x1)
    z_by_y = lambda y: dZdY * (y - y1)
    while (intersect == 0) and cur_voxel != target_voxel:
        bnds = cur_voxel + B
        distXY = points_sqr_len(np.array([x_by_z(bnds[2]), y_by_z(bnds[2]), bnds[2]]), cur_voxel)
        distYZ = points_sqr_len(np.array([bnds[0], y_by_x(bnds[0]), z_by_x(bnds[0])]), cur_voxel)
        distXZ = points_sqr_len(np.array([x_by_y(bnds[1]), bnds[1], z_by_y(bnds[1])]), cur_voxel)

        if (distXY <= distXZ) and (distXY <= distYZ):
            cur_voxel[2] += sZ
        if (distXZ <= distXY) and (distXZ <= distYZ):
            cur_voxel[1] += sY
        if (distYZ <= distXZ) and (distYZ <= distXY):
            cur_voxel[2] += sZ
        
        if occupancy_grid[cur_voxel[0], cur_voxel[1], cur_voxel[2]]:
            intersect = 1
    
    return intersect

def same_point_condition(current_point, target_point):
    return current_point == target_point

def point_in_range_condition(current_point, target_point):
    return max_observe_dist > points_sqr_len(current_point, target_point) * voxel_size

#def find_path_A_star(grid, start, end, stop_condition)

def main():
    pcd = o3d.io.read_point_cloud(input_path)
    print("Yellow sphere is the start point, Blue sphere is the target point")
    mission_points_ids = list(set(pick_points(pcd)))
    assert (len(mission_points_ids) == 2), "It must be exactly 2 picked points"

    target_point = np.asarray(pcd.points)[mission_points_ids[0]]
    start_point = np.asarray(pcd.points)[mission_points_ids[1]]

    pcd = pcd.voxel_down_sample(voxel_size)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    target_voxel = voxel_grid.get_voxel(target_point)
    start_voxel = voxel_grid.get_voxel(start_point)

    voxels = voxel_grid.get_voxels()
    boundary = get_max_bounding_idx(pcd)
    occupancy_grid = get_occupancy_grid(boundary, voxels)




if __name__ == '__main__':
    main()
