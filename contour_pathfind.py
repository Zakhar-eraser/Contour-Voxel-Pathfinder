import numpy as np
import open3d as o3d
import pathfinder.A_star_pathfinder as asp
import open3d.visualization.gui as gui
import visualizer.visualizer as visualizer
import utils.map_manager as mm
import menu.console_menu as menu
import structures.destination_list as dl
from utils.grids.occupancy_grid import pos2idx

input_path = 'cloud2dfa7db512d9b041 - Cloud.ply'
voxel_size = 1.0

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
    info = menu.open_or_create_project_dialogue()
    if info.project_dir is not None:
        pcd = o3d.io.read_point_cloud(info.project_dir + mm.pc_file)
        min_bound = pcd.get_min_bound()
        scene = visualizer.PointsSelectorApp(pcd)
        app.run()
        position = scene.targets
        start_height = position.mark.get_center()[2]  ## DO NOT DELETE
        assert position is not None and position.target is not None, "It must be set 2 points atleast"
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

        with open(info.project_dir + mm.map_occupancy_grid, 'rb') as grid:
            occupancy_grid = np.load(grid)

        line_sets = []
        marks = [position.mark]
        while position.target is not None:
            marks += [position.target.mark]
            if position.target.transfer == dl.Transfer.DESTINATE:
                stop = asp.same_point_cond
                mp = lambda c, n: 0
                last_color = [1, 0, 0]
            else:
                stop = asp.visibility_cond
                mp = asp.plane_move_priority
                last_color = [0, 0, 1]
            target_voxel = pos2idx(min_bound, position.target.mark.get_center())
            if position.transfer == dl.Transfer.DESTINATE or position.transfer == None:
                start_voxel = pos2idx(min_bound, position.mark.get_center())
            else:
                start_voxel = route[1] + 1
            tmp = occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)]
            occupancy_grid[tuple(start_voxel)] = occupancy_grid[tuple(target_voxel)] = 0
            graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, stop, mp)
            occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)] = tmp
            route = asp.get_route(graph, min_bound, target_voxel)
            #mm.write_waypoints(project_dir, position.target.name + "_route"
            #    , route + info.shift - (0, 0, start_height))
            line_sets += [make_route_lines(route, last_color)]
            position = position.target

        o3d.visualization.draw_geometries([voxel_grid] + line_sets + marks)

if __name__ == '__main__':
    main()
