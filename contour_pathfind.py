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

def same_point_condition(current_point, target_point):
    return np.array_equal(current_point, target_point)

def observable_position(current_point, target_point):
    return

def draw_route(voxel_grid, route, min_bound):
    points = route * voxel_size + min_bound
    lines = np.arange(len(points) - 1)[:, np.newaxis]
    lines = np.concatenate((lines, lines + 1), axis=1)
    colors = [[1, 0, 0] for i in range(len(lines) - 1)]
    colors.append([0, 0, 1])
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([voxel_grid, line_set])

def main():
    pcd = o3d.io.read_point_cloud(input_path)
    mission_points_ids = list(set(pick_points(pcd)))
    assert (len(mission_points_ids) == 2), "It must be exactly 2 picked points"
    target_point = np.asarray(pcd.points)[mission_points_ids[0]]
    start_point = np.asarray(pcd.points)[mission_points_ids[1]]

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

    graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel)
    route = asp.get_route_idx(graph, target_voxel)
    draw_route(voxel_grid, route, pcd.get_min_bound())


if __name__ == '__main__':
    main()
