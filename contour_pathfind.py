import numpy as np
import open3d as o3d
import A_star_pathfinder as asp

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

def get_max_shape_idx(pcd):
    max_bound = pcd.get_max_bound()
    min_bound = pcd.get_min_bound()
    shape = (max_bound - min_bound) / voxel_size + np.ones(3, dtype=np.int32)
    shape = shape.astype('int32')
    return shape

def get_occupancy_grid(shape, voxels):
    grid = np.zeros(tuple(shape + 2), dtype=np.int8)
    grid[:, :, 0] = 1
    grid[:, :, shape[2] + 1] = 1
    grid[0, :, 1:shape[2] + 1] = 1
    grid[shape[0] + 1, :, 1:shape[2] + 1] = 1
    grid[1:shape[0] + 1, 0, 1:shape[2] + 1] = 1
    grid[1:shape[0] + 1, shape[1] + 1, 1:shape[2] + 1] = 1
    
    for vox in voxels:
        grid[tuple(vox.grid_index + 1)] = 1
    return grid

def points_sqr_len(p1, p2):
    dp = p1 - p2
    return dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]

def check_intersection(start_voxel, target_voxel, occupancy_grid):
    tmp = occupancy_grid[target_voxel[0], target_voxel[1], target_voxel[2]]
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
    B = np.full((1, 3), fill_value=vox_step, dtype=np.float32)
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
            cur_voxel[0] += sX
        
        if occupancy_grid[cur_voxel[0], cur_voxel[1], cur_voxel[2]]:
            intersect = 1
    occupancy_grid[target_voxel[0], target_voxel[1], target_voxel[2]] = tmp
    return intersect

def same_point_condition(current_point, target_point):
    return np.array_equal(current_point, target_point)

def point_in_range_condition(current_point, target_point):
    return max_observe_dist > points_sqr_len(current_point, target_point) * voxel_size

def draw_route(voxel_grid, route, min_bound):
    points = route * voxel_size + min_bound
    lines = np.arange(len(points) - 1)[:, np.newaxis]
    lines = np.concatenate((lines, lines + 1), axis=1)
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([voxel_grid, line_set])

def main():
    pcd = o3d.io.read_point_cloud(input_path)
    print("Yellow sphere is the start point, Blue sphere is the target point")
    mission_points_ids = list(set(pick_points(pcd)))
    assert (len(mission_points_ids) == 2), "It must be exactly 2 picked points"
    target_point = np.asarray(pcd.points)[mission_points_ids[0]]
    start_point = np.asarray(pcd.points)[mission_points_ids[1]]
    ##target_point = np.array((32, -2, 92))
    ##start_point = np.array((34, -17, 92))

    pcd = pcd.voxel_down_sample(voxel_size)
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    target_voxel = voxel_grid.get_voxel(target_point)
    start_voxel = voxel_grid.get_voxel(start_point)

    start_voxel += 1
    target_voxel += 1
    voxels = voxel_grid.get_voxels()
    shape = get_max_shape_idx(pcd)
    occupancy_grid = get_occupancy_grid(shape, voxels)
    occupancy_grid[tuple(target_voxel)] = 0
    occupancy_grid[tuple(start_voxel)] = 0

    graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, same_point_condition)
    route = asp.get_route_idx(graph, target_voxel)
    draw_route(voxel_grid, route, pcd.get_min_bound())


if __name__ == '__main__':
    main()
