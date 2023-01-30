#!/venv/bin/python
import open3d as o3d
from os.path import join
import pathfinder.A_star_pathfinder as asp
import open3d.visualization.gui as gui
from visualizer.SelectPointsWindow import PointsSelectorApp
from visualizer.OutputWindow import OutputWindow
import utils.map_manager as mm
import menu.console_menu as menu
from structures.destination_list import Transfer
from structures.destination_list import get_writeble_targets
from utils.grids.occupancy_grid import qpos2idx
from utils.grids.occupancy_grid import set_static_min_bound
from utils.grids.occupancy_grid import set_static_voxel_size
from structures.route import Route
from structures.route import route2array
from visualizer.common.visualizer_geometry import get_targets

def graph2list(graph, end):
    cur = end
    route = []
    while graph[tuple(cur)] is not None:
        route.append(cur)
        cur = graph[tuple(cur)]
    route.append(cur)
    route.reverse()
    return route

def main():
    mm.init_project_structure()
    info = menu.open_or_create_project_dialogue()
    if info.project_dir is not None:
        pcd = o3d.io.read_point_cloud(join(info.project_dir, mm.pc_file))
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, info.voxel_size)
        vs = voxel_grid.voxel_size
        set_static_min_bound(voxel_grid.get_min_bound())
        set_static_voxel_size(vs)
        occupancy_grid = mm.read_grid(info.project_dir)
        app = gui.Application.instance
        app.initialize()
        select_scene = PointsSelectorApp(voxel_grid, occupancy_grid, get_targets(mm.read_targets(info.project_dir), vs/2 + 0.1))
        app.run()
        targets = select_scene.targets
        assert targets is not None and targets.target is not None, "It must be set 2 points atleast"
        mm.write_targets(info.project_dir, get_writeble_targets(targets))
        start_height = select_scene.start_height

        route_idx_root = Route()
        route_idx_root.point = qpos2idx(targets.mark.get_center())
        route_idx = route_idx_root
        while targets.target is not None:
            if targets.target.transfer == Transfer.OBSERVE:
                stop = asp.visibility_cond
                asp.H = 70
            else:
                stop = asp.same_point_cond
                asp.H = 1
            target_voxel = targets.target.idx
            if targets.transfer == Transfer.VISIT or targets.transfer == Transfer.START:
                start_voxel = targets.idx
            else:
                start_voxel = route_idx.point
            tmp = occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)]
            occupancy_grid[tuple(start_voxel)] = occupancy_grid[tuple(target_voxel)] = 0
            graph = asp.find_path_A_star(occupancy_grid, start_voxel, target_voxel, stop)
            occupancy_grid[tuple(start_voxel)], occupancy_grid[tuple(target_voxel)] = tmp
            path_idx = graph2list(graph, target_voxel)
            obs_point = None
            if targets.target.transfer == Transfer.OBSERVE:
                obs_point = path_idx.pop()
            if len(path_idx) > 1:
                if obs_point is not None: route_idx.observe_points.append(obs_point)
                route_idx.next_point = Route()
                route_idx.next_point.prev_point = route_idx
                path_idx.pop(0)
                route_idx.next_point.point = path_idx.pop()
                route_idx.visit_points += path_idx
                route_idx = route_idx.next_point
            else:
                route_idx.prev_point.observe_points.append(obs_point)
            targets = targets.target
        
        output_scene = OutputWindow(voxel_grid, occupancy_grid, route_idx_root, select_scene.targets)
        app.run()
        route = output_scene.route
        menu.use_utm_coordinates_dialogue()
        menu.write_points_dialogue(info.project_dir, "mission",
            route2array(route) + (info.shift[0], info.shift[1], -start_height))

if __name__ == '__main__':
    main()
