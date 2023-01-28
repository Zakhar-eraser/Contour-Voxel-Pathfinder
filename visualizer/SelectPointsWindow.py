import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
from structures.destination_list import Targets
from structures.destination_list import Transfer
from structures.destination_list import exists_idx
import utils.voxel_raycast as vr
from utils.grids.occupancy_grid import qidx2pos
from utils.grids.occupancy_grid import qpos2idx
from utils.geometry.line_plane_intersection import find_intersection
from utils.geometry.distances import sqr_dist
from visualizer.common.visualizer_geometry import MarkerColors
from visualizer.common.visualizer_geometry import create_mark
from visualizer.common.visualizer_geometry import create_line
from copy import deepcopy

class PointsSelectorApp:

    def __init__(self, voxel_grid, occupancy_grid, targets = None):
        vx = voxel_grid
        self._grid = occupancy_grid
        self._grid_with_markers = deepcopy(occupancy_grid)
        app = gui.Application.instance
        self._window = app.create_window("Select points", 1024, 640)
        self._widget3d = gui.SceneWidget()
        self._window.add_child(self._widget3d)
        self._info = gui.Label("Select start position")
        self._info.visible = True
        self._window.add_child(self._info)

        # Setting gui`s panel
        em = self._window.theme.font_size
        separation_height = int(round(0.5 * em))
        self._settings_panel = gui.Vert(
            0, gui.Margins(0.25 * em, 0.25 * em, 0.25 * em, 0.25 * em))
        
        # adding checkbox on gui`s panel
        self._checkbox = gui.Checkbox("Stop when observable")
        self._checkbox.set_on_checked(self._on_checkbox)
        self._settings_panel.add_fixed(separation_height)
        self._settings_panel.add_child(self._checkbox)

        # adding 'calculate' button on gui`s panel
        self._calc_button = gui.Button("Calculate")
        self._calc_button.set_on_clicked(self._on_calc_clicked)
        self._settings_panel.add_fixed(separation_height)
        self._settings_panel.add_child(self._calc_button)

        self._window.add_child(self._settings_panel)

        self._window.set_on_layout(self._on_layout)

        self._widget3d.scene = rendering.Open3DScene(self._window.renderer)
        self._widget3d.scene.set_background((0.8, 0.6, 0.6, 1))

        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"

        mat.point_size = 6 * self._window.scaling
        self._widget3d.scene.add_geometry("Point Cloud", vx, mat)

        bounds = self._widget3d.scene.bounding_box
        center = bounds.get_center()
        self._widget3d.setup_camera(60, bounds, center)

        self._widget3d.set_on_mouse(self._on_mouse_widget3d)
        self._widget3d.set_on_key(self._on_keyboard_widget3d)
        self.targets = targets
        self._last_target = self.targets
        self._selected_target = None
        self._transfer_type = Transfer.VISIT
        self.start_height = 0

        self._vs = vx.voxel_size
        self._bounding_box = vx.get_axis_aligned_bounding_box()
        self._shape = (self._bounding_box.get_extent() /
            self._vs).astype('int32')
    
    def _idx_in_bounds(self, idx, min, max):
        in_bounds = False
        if (idx[0] >= min[0]) and (idx[1] >= min[1]) and (idx[2] >= min[2]) and (
            (idx[0] <= max[0]) and (idx[1] <= max[1]) and (idx[2] <= max[2])):
            in_bounds = True
        return in_bounds
    
    def _get_nearest_bounding_box_voxel(self, line):
        s = self._shape
        if (self._idx_in_bounds(line[0], (0, 0, 0), s)):
            voxel = line[0]
        else:
            planes = np.array((((1, 1, 1), (1, 1, s[2]), (1, s[1], s[2])),
                      ((1, 1, 1), (s[0], 1, 1),(s[0], 1, s[2])),
                      ((1, 1, 1), (1, s[1], 1), (s[0], s[1], 1)),
                      ((s[0], 1, 1), (s[0], s[1], 1), s),
                      ((1, s[1], 1), (s[0], s[1], 1), s),
                      ((1, 1, s[2]), (s[0], 1, s[2]), s)))
            
            min_dist = float('inf')
            voxel = None
            for p in planes:
                target = find_intersection(line, p)
                if (target is not None):
                    target = target.astype('int32')
                    if self._idx_in_bounds(target, p[0], p[2]):
                        dist = sqr_dist(target, line[0])
                        if (dist < min_dist):
                            min_dist = dist
                            voxel = target
        return voxel
    
    def _on_layout(self, layout_context):
        r = self._window.content_rect
        self._widget3d.frame = r
        pref = self._info.calc_preferred_size(layout_context,
                                             gui.Widget.Constraints())
        
        self._info.frame = gui.Rect(r.x,
                                   r.get_bottom() - pref.height, pref.width,
                                   pref.height)
        
        width = 17 * layout_context.theme.font_size
        height = min(
            r.height,
            self._settings_panel.calc_preferred_size(
                layout_context, gui.Widget.Constraints()).height)
        self._settings_panel.frame = gui.Rect(r.get_right() - width, r.y, width,
                                              height)
    
    def _on_checkbox(self, show):
        if show:
            self._transfer_type = Transfer.OBSERVE
        else:
            self._transfer_type = Transfer.VISIT
    
    def _on_calc_clicked(self):
        gui.Application.instance.quit()
    
    def _move_marker(self, target, shift):
        idx = tuple(target.idx)
        self._grid_with_markers[idx] = self._grid[idx]
        target.move(shift)
        self._grid_with_markers[tuple(target.idx)]
    
    def _update_line(self, target, material):
        origin = target.origin
        name = target.name
        self._widget3d.scene.remove_geometry(name)
        line = create_line(origin.mark.get_center(),
                           target.mark.get_center(),
                           target.transfer)
        self._widget3d.scene.add_geometry(name, line, material)

    
    def _update_marker_lines(self, target):
        mat = rendering.MaterialRecord()
        mat.shader = "unlitLine"
        mat.line_width = 5
        self._update_line(target, mat)
        if target.target is not None: self._update_line(target.target, mat)

    def _on_keyboard_widget3d(self, event):
        selected = self._selected_target
        if event.type == gui.KeyEvent.Type.DOWN:
            if selected is not None:
                mat = rendering.MaterialRecord()
                mat.shader = "defaultLit"
                name = selected.name
                if event.key == gui.KeyName.ENTER:
                    self._selected_target = None
                    self._widget3d.scene.remove_geometry(name)
                    if selected.transfer == Transfer.START:
                        transfer = Transfer.START
                        selected.add(Targets(
                            create_mark(
                                selected.mark.get_center() + (0, 0, self._vs), self._vs / 2 + 0.1, Transfer.VISIT),
                                "mark_1", 1), Transfer.VISIT)
                        takeoff_marker = selected.target
                        self._selected_target = takeoff_marker
                        self._widget3d.scene.add_geometry(takeoff_marker.name, takeoff_marker.mark, mat)
                    else:
                        transfer = selected.transfer
                        self._update_marker_lines(selected, mat) 

                    transfer = self._transfer_type
                    selected.mark.paint_uniform_color(MarkerColors.get_color(transfer))
                    selected.transfer = transfer
                    geometry = selected.mark
                    self._info.text = "Adding new target"
                else:
                    mat.shader = "defaultLit"
                    geometry = self._selected_target.mark
                    name = self._selected_target.name
                    if event.key == gui.KeyName.W:
                        self._move_marker(self._selected_target, (0, 0, self._vs))
                    elif event.key == gui.KeyName.S:
                        self._move_marker(self._selected_target, (0, 0, -self._vs))
                    elif event.key == gui.KeyName.LEFT and self._selected_target.id != 1:
                        self._move_marker(self._selected_target, (-self._vs, 0, 0))
                    elif event.key == gui.KeyName.RIGHT and self._selected_target.id != 1:
                        self._move_marker(self._selected_target, (self._vs, 0, 0))
                    elif event.key == gui.KeyName.UP and self._selected_target.id != 1:
                        self._move_marker(self._selected_target, (0, self._vs, 0))
                    elif event.key == gui.KeyName.DOWN and self._selected_target.id != 1:
                        self._move_marker(self._selected_target, (0, -self._vs, 0))
                    else:
                        return gui.Widget.EventCallbackResult.IGNORED

                    if self._selected_target.origin is None:
                        self.start_height = height = geometry.get_center()[2]
                    else:
                        height = geometry.get_center()[2] - self.start_height
                    self._info.text = "Height: {:.2f}".format(height)
                    self._widget3d.scene.remove_geometry(name)
                
                self._window.set_needs_layout()
                self._widget3d.scene.add_geometry(name, geometry, mat)
                return gui.Widget.EventCallbackResult.HANDLED
            else:
                if event.key == gui.BACKSPACE and self._last_target is not None:
                    self._widget3d.scene.remove_geometry(self._last_target.name + "_line")
                    self._widget3d.scene.remove_geometry(self._last_target.name)
                    self._last_target = self._last_target.origin
                    if self._last_target is not None:
                        self._last_target.target = None
                    else:
                        self.targets = None
                else:
                    return gui.Widget.EventCallbackResult.IGNORED 
                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED

    def _raycast(self, event, grid):
        cam_pos = self._widget3d.scene.camera.unproject(
            event.x, event.y, 0, self._widget3d.frame.width,
            self._widget3d.frame.height)
        cam_vox = qpos2idx(cam_pos)
        dist_pos = self._widget3d.scene.camera.unproject(
            event.x, event.y, 0.99999, self._widget3d.frame.width,
            self._widget3d.frame.height)
        dist_vox = qpos2idx(dist_pos)
        bnd_vox = self._get_nearest_bounding_box_voxel((cam_vox, dist_vox))
        return vr.raycast(bnd_vox, dist_vox, grid)

    def _on_mouse_widget3d(self, event):
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and self._selected_target is None:
            if event.is_modifier_down(gui.KeyModifier.CTRL):
                target_vox = self._raycast(event, self._grid)
                if len(target_vox):
                    world = qidx2pos(target_vox[0])

                    mark = create_mark(world, self._vs / 2 + 0.1, MarkerColors.SELECTED)
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"

                    if self.targets is None:
                        mark.translate((0, 0, self._vs / 2 + 0.001))
                        self.start_height = height = mark.get_center()[2]
                        name = "start"
                        self._last_target = self.targets = Targets(mark, name, 0)
                        self._last_target.transfer = Transfer.START
                    else:
                        height = world[2] - self.start_height
                        name = "mark_" + str(self._last_target.id + 1)
                        self._last_target = self._last_target.add(Targets(mark, name, self._last_target.id + 1),
                            self._transfer_type)

                    self._info.text = "Height: {:.2f}".format(height)
                    self._window.set_needs_layout()

                    self._grid_with_markers[tuple(self._last_target.idx)] = 1
                    self._widget3d.scene.add_geometry(name, mark, mat)

                    self._selected_target = self._last_target

                    return gui.Widget.EventCallbackResult.HANDLED
            elif event.is_modifier_down(gui.KeyModifier.SHIFT):
                target_vox = self._raycast(event, self._grid_with_markers)
                editing_target = None
                for vox in target_vox:
                    editing_target = exists_idx(self.targets, vox)
                    if editing_target is not None: break
                if editing_target is not None:
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"
                    self._selected_target = editing_target
                    self._widget3d.scene.remove_geometry(editing_target.name)
                    editing_target.mark.paint_uniform_color(MarkerColors.SELECTED)
                    self._widget3d.scene.add_geometry(editing_target.name, editing_target.mark, mat)
            else:
                return gui.Widget.EventCallbackResult.IGNORED