import numpy as np
import open3d as o3d
import A_star_pathfinder as asp
import open3d.visualization.gui as gui
import visualizer
import map_manager as mm

input_path = 'campus - CloudClean sub.ply'
voxel_size = 1.0

def get_max_shape_idx(min_bound, max_bound):
    shape = (max_bound - min_bound) / voxel_size + 1
    shape = shape.astype('int32')
    return shape

def pos_to_idx(min_bound, pos):
    return ((pos - min_bound) / voxel_size).astype('int32')

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

def make_route(voxel_grid, route, min_bound):
    points = route * voxel_size + min_bound
    lines = np.arange(len(points) - 1)[:, np.newaxis]
    lines = np.concatenate((lines, lines + 1), axis=1)
    colors = [[1, 0, 0] for i in range(len(lines) - 1)]
    colors.insert(0, [0, 0, 1])
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def main():
    app = gui.Application.instance
    app.initialize()
    pcd = o3d.io.read_point_cloud(mm.create_project(input_path, voxel_size))
    scene = visualizer.PointsSelectorApp(pcd)
    app.run()
    mission_points = scene.targets
    max_bound = pcd.get_max_bound()
    min_bound = pcd.get_min_bound()
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)
    start_voxel = pos_to_idx(min_bound, mission_points[0]) + 1
    target_voxel = pos_to_idx(min_bound, mission_points[1]) + 1

    voxels = voxel_grid.get_voxels()
    shape = get_max_shape_idx(min_bound, max_bound)
    occupancy_grid = get_occupancy_grid(shape, voxels)
    occupancy_grid[tuple(target_voxel)] = 0
    occupancy_grid[tuple(start_voxel)] = 0

    assert mission_points is not None and mission_points.target is not None, "It must be set 2 points atleast"
    line_sets = []
    marks = [mission_points.mark]
    while mission_points.target is not None:
        graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, asp.visibility_cond)
        route = asp.get_route_idx(graph, target_voxel)
        line_sets += make_route(voxel_grid, route, min_bound)

        mission_points

    o3d.visualization.draw_geometries([voxel_grid, line_set])


if __name__ == '__main__':
    main()
