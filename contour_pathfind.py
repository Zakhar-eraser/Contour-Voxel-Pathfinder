#!/venv/bin/python
import numpy as np
import open3d as o3d
from os.path import join
import pathfinder.A_star_pathfinder as asp
import open3d.visualization.gui as gui
from visualizer.SelectPointsWindow import PointsSelectorApp
import utils.map_manager as mm
import menu.console_menu as menu
from structures.destination_list import Transfer
from utils.grids.occupancy_grid import pos2idx
from structures.route import Route
from structures.route import route_idx2pos
from structures.route import route2array
from structures.route import targets2route
from visualizer.common.visualizer_geometry import route2targets

def graph2list(graph, end):
    cur = end
    route = []
    while graph[tuple(cur)] is not None:
        route.append(cur)
        cur = graph[tuple(cur)]
    route.append(cur)
    route.reverse()
    return route

#def optimize(route_idx, min_hor_idx_cnt):
#    route_idx_opt = []
#    prev_line_idx = []
#    cur_line_idx = []
#    mode = 1
#    for _ in range(len(route_idx)):
#        if mode == 0:
#            
#            if route_idx[]

def main():
    mm.init_project_structure()
    info = menu.open_or_create_project_dialogue()
    if info.project_dir is not None:
        pcd = o3d.io.read_point_cloud(join(info.project_dir, mm.pc_file))
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, info.voxel_size)
        vs = voxel_grid.voxel_size
        min_bound = voxel_grid.get_min_bound()
        occupancy_grid = mm.read_grid(info.project_dir)
        app = gui.Application.instance
        app.initialize()
        scene = PointsSelectorApp(voxel_grid, occupancy_grid, route2targets(mm.read_targets(info.project_dir), vs + 0.1))
        app.run()
        targets = scene.targets
        assert targets is not None and targets.target is not None, "It must be set 2 points atleast"
        #mm.write_targets(info.project_dir, targets2route(targets))
        start_height = targets.mark.get_center()[2]  ## vs / 2 means the takeoff altitude

        route_idx_root = Route()
        route_idx_root.point = pos2idx(min_bound, targets.mark.get_center(), vs)
        route_idx = route_idx_root
        while targets.target is not None:
            if targets.target.transfer == Transfer.DESTINATE:
                stop = asp.same_point_cond
                asp.H = 1
            else:
                stop = asp.visibility_cond
                asp.H = 70
            target_voxel = pos2idx(min_bound,
                targets.target.mark.get_center(),
                vs)
            if targets.transfer == Transfer.DESTINATE or targets.transfer == None:
                start_voxel = pos2idx(min_bound, targets.mark.get_center(), vs)
            else:
                start_voxel = route_idx.point
            tmp = occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)]
            occupancy_grid[tuple(start_voxel)] = occupancy_grid[tuple(target_voxel)] = 0
            graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, stop)
            occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)] = tmp
            path_idx = graph2list(graph, target_voxel)
            if targets.target.transfer == Transfer.OBSERVE:
                route_idx.observe_points += path_idx.pop()
            if len(path_idx) > 1:
                route_idx.next_point = Route()
                path_idx.pop(0)
                route_idx.next_point.point = path_idx.pop()
                route_idx.visit_points += path_idx
                route_idx = route_idx.next_point
            targets = targets.target
        
        route = route_idx2pos(min_bound, route_idx_root, vs)
        menu.use_utm_coordinates_dialogue()
        menu.write_points_dialogue(info.project_dir, "mission",
            route2array(route) + (info.shift[0], info.shift[1], -start_height))

if __name__ == '__main__':
    main()
