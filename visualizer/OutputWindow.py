import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
import structures.destination_list as dl
import utils.voxel_raycast as vr
from utils.grids.occupancy_grid import pos2idx
from utils.grids.occupancy_grid import idx2pos
from utils.geometry.line_plane_intersection import find_intersection
from utils.geometry.distances import sqr_dist

class OutputWindow:

    def __init__(self, voxel_grid, occupancy_grid):
        vx = voxel_grid
        self._grid = occupancy_grid

        app = gui.Application.instance
        self.window = app.create_window("New window", 1024, 640)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)

        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene.set_background((0.8, 0.6, 0.6, 1))

        bounds = self.widget3d.scene.bounding_box
        center = bounds.get_center()
        self.widget3d.setup_camera(60, bounds, center)

