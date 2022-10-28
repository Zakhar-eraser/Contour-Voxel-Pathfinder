import numpy as np
import open3d as o3d
import A_star_pathfinder as asp
import open3d.visualization.gui as gui
import visualizer
import map_manager as mm
import destination_list as dl

input_path = 'cloud2dfa7db512d9b041 - Cloud.ply'
voxel_size = 1.0

def get_max_shape_idx(min_bound, max_bound):
    shape = (max_bound - min_bound) / voxel_size + 1
    shape = shape.astype('int32')
    return shape

def pos_to_idx(min_bound, pos):
    return ((pos - min_bound) / voxel_size).astype('int32') + 1

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

def make_route_lines(route, last_color):
    points = route * voxel_size
    lines = np.arange(len(points) - 1)[:, np.newaxis]
    lines = np.concatenate((lines, lines + 1), axis=1)
    colors = [[1, 0, 0] for i in range(len(lines) - 1)]
    colors.insert(0, last_color)
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines))
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def main():
    app = gui.Application.instance
    app.initialize()
    project_dir = mm.create_project(input_path, voxel_size)
    pcd = o3d.io.read_point_cloud(project_dir + mm.pc_file)
    max_bound = pcd.get_max_bound()
    min_bound = pcd.get_min_bound()
    shift = mm.get_map_shift(project_dir)
    scene = visualizer.PointsSelectorApp(pcd)
    app.run()
    position = scene.targets
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    voxels = voxel_grid.get_voxels()
    shape = get_max_shape_idx(min_bound, max_bound)
    occupancy_grid = get_occupancy_grid(shape, voxels)

    assert position is not None and position.target is not None, "It must be set 2 points atleast"
    line_sets = []
    marks = [position.mark]
    while position.target is not None:
        marks += [position.target.mark]
        if position.target.transfer == dl.Transfer.DESTINATE:
            stop = asp.same_point_cond
            mp = asp.euclidian_priority
            last_color = [1, 0, 0]
        else:
            stop = asp.visibility_cond
            mp = asp.plane_move_priority
            last_color = [0, 0, 1]
        target_voxel = pos_to_idx(min_bound, position.target.mark.get_center())
        if position.transfer == dl.Transfer.DESTINATE or position.transfer == None:
            start_voxel = pos_to_idx(min_bound, position.mark.get_center())
        else:
            start_voxel = route[1] + 1
        tmp = occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)]
        occupancy_grid[tuple(start_voxel)] = occupancy_grid[tuple(target_voxel)] = 0
        graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, stop, mp)
        occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)] = tmp
        route = asp.get_route(graph, min_bound, target_voxel)
        mm.write_waypoints(project_dir, position.target.name + "_route"
            , route + shift)
        line_sets += [make_route_lines(route, last_color)]
        position = position.target

    o3d.visualization.draw_geometries([voxel_grid] + line_sets + marks)

if __name__ == '__main__':
    main()
