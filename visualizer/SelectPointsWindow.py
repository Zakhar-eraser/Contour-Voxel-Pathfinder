import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
from structures.destination_list import Targets
from structures.destination_list import Transfer
from structures.destination_list import exists_idx
from structures.destination_list import end
import utils.voxel_raycast as vr
from utils.grids.occupancy_grid import qidx2pos
from utils.grids.occupancy_grid import qpos2idx
from utils.geometry.line_plane_intersection import find_intersection
from utils.geometry.distances import sqr_dist
from visualizer.common.visualizer_geometry import MarkerColors
from visualizer.common.visualizer_geometry import create_mark
from visualizer.common.visualizer_geometry import create_line
from copy import deepcopy

class Materials:
    MARKER = 1
    LINE = 2
    VOXEL = 3

class PointsSelectorApp:
    _START_MARKER = 'start'
    _TAKEOFF_MARKER_NAME = 'mark_1'
    _MARKER_PREFIX = 'mark_'
    _LINE_SUFFIX = '_line'

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

        self.materials = self._initialize_materials()

        self._widget3d.scene.add_geometry("Voxel map", vx, self.materials[Materials.VOXEL])

        bounds = self._widget3d.scene.bounding_box
        center = bounds.get_center()
        self._widget3d.setup_camera(60, bounds, center)

        self._widget3d.set_on_mouse(self._on_mouse_widget3d)
        self._widget3d.set_on_key(self._on_keyboard_widget3d)
        self.targets = targets
        self._last_id = 0
        self._spawn_targets_geometry()
        self._last_target = end(targets)
        self._selected_target = None
        self._transfer_type = Transfer.VISIT
        self.start_height = 0
        if targets is not None: self.start_height = targets.mark.get_center()[2] 

        self._vs = vx.voxel_size
        self._bounding_box = vx.get_axis_aligned_bounding_box()
        self._shape = (self._bounding_box.get_extent() /
            self._vs).astype('int32')
    
    def _initialize_materials(self) -> dict:
        line_mat = rendering.MaterialRecord()
        line_mat.shader = "unlitLine"
        line_mat.line_width = 5

        marker_mat = rendering.MaterialRecord()
        marker_mat.shader = "defaultLit"

        voxel_mat = rendering.MaterialRecord()
        voxel_mat.shader = "defaultUnlit"
        voxel_mat.point_size = 6 * self._window.scaling

        return {Materials.MARKER: marker_mat,
                Materials.LINE: line_mat,
                Materials.VOXEL: voxel_mat}
    
    def _spawn_targets_geometry(self):
        tgt = self.targets
        if tgt is not None:
            self._grid_with_markers[tuple(tgt.idx)] += 1
            self._widget3d.scene.add_geometry(tgt.name, tgt.mark, marker_mat)
            while tgt.target is not None:
                self._grid_with_markers[tuple(tgt.target.idx)] += 1
<<<<<<< HEAD
                self._widget3d.scene.add_geometry(tgt.target.name, tgt.target.mark, marker_mat)
                line = create_line(tgt.mark.get_center(), tgt.target.mark.get_center(),
=======
                self._last_id += 1
                name = PointsSelectorApp._MARKER_PREFIX + str(self._last_id)
                self._widget3d.scene.add_geometry(name, tgt.target.mark, marker_mat)
                line = create_line(tgt.pos, tgt.target.pos,
>>>>>>> 28f38d0 (targets id allocation fix init. development freezed)
                    MarkerColors.get_color(tgt.target.transfer))
                self._widget3d.scene.add_geometry(name + PointsSelectorApp._LINE_SUFFIX,
                                                  line, line_mat)
                tgt = tgt.target

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
        pos = target.mark.get_center() + shift
        new_idx = qpos2idx(pos)
        if self._idx_in_bounds(new_idx, (0, 0, 0), self._shape):
            self._grid_with_markers[tuple(target.idx)] -= 1
            target.mark.translate(pos, False)
            target.idx = new_idx
            self._grid_with_markers[tuple(target.idx)] += 1
    
    def _destroy_line(self, target: Targets) -> None:
        if target.origin is not None:
            self._widget3d.scene.remove_geometry(target.name + PointsSelectorApp._LINE_SUFFIX)
    
    def _spawn_line(self, target: Targets) -> None:
        origin = target.origin
        name = target.name + "_line"
        self._widget3d.scene.remove_geometry(name)
        line = create_line(origin.mark.get_center(),
                           target.mark.get_center(),
                           MarkerColors.get_color(target.transfer))
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
                geometry = selected.mark
                if event.key == gui.KeyName.ENTER:
                    self._selected_target = None
                    if selected.transfer == Transfer.START:
                        selected.transfer = Transfer.START
                        if selected.target is None:
                            selected.add(Targets(
                                create_mark(
                                    selected.mark.get_center() + (0, 0, self._vs / 2),
                                    self._vs / 2 + 0.1, MarkerColors.SELECTED),
                                    "mark_1", 1), Transfer.TAKEOFF)
                        else:
                            self._widget3d.scene.remove_geometry(selected.target.name)
                            self._widget3d.scene.remove_geometry(selected.target.name + "_line")
                            selected.target.mark.translate(selected.mark.get_center() + (0, 0, self._vs / 2), False)
                            selected.target.mark.paint_uniform_color(MarkerColors.SELECTED)
                        takeoff_marker = selected.target
                        self._grid_with_markers[tuple(takeoff_marker.idx)] += 1
                        self._widget3d.scene.add_geometry(takeoff_marker.name, takeoff_marker.mark, mat)
                        self._last_target = takeoff_marker
                        self._selected_target = takeoff_marker
                    else:
                        if selected.transfer == Transfer.TAKEOFF:
                            selected.transfer = Transfer.TAKEOFF
                        else:
                            selected.transfer = self._transfer_type
                        self._update_marker_lines(selected) 

                    selected.mark.paint_uniform_color(MarkerColors.get_color(selected.transfer))
                    self._info.text = "Adding new target"
                else:
                    mat.shader = "defaultLit"
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
                
                self._window.set_needs_layout()
                self._widget3d.scene.remove_geometry(name)
                self._widget3d.scene.add_geometry(name, geometry, mat)
                return gui.Widget.EventCallbackResult.HANDLED
            else:
                if event.key == gui.BACKSPACE and self._last_target is not None:
                    self._widget3d.scene.remove_geometry(self._last_target.name + "_line")
                    self._widget3d.scene.remove_geometry(self._last_target.name)
                    self._grid_with_markers[tuple(self._last_target.idx)] -= 1
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
                if self._last_target is not None and self._last_target.origin is None:
                    target_vox = [self._last_target.idx + (0, 0, 1)]
                else: target_vox = self._raycast(event, self._grid_with_markers)
                if len(target_vox):
                    world = qidx2pos(target_vox[0])
                    mark = create_mark(world, self._vs / 2 + 0.1, MarkerColors.SELECTED)
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"

                    if self.targets is None:
                        mark.translate((0, 0, self._vs / 2 + 0.001))
                        self.start_height = height = mark.get_center()[2]
                        name = PointsSelectorApp._START_MARKER
                        self._last_target = self.targets = Targets(mark, name)
                        self._last_target.transfer = Transfer.START
                    else:
                        height = world[2] - self.start_height
                        name = "mark_" + str(self._last_target.id + 1)
                        self._last_target = self._last_target.add(Targets(mark, name, self._last_target.id + 1),
                            self._transfer_type)

                    self._info.text = "Height: {:.2f}".format(height)
                    self._window.set_needs_layout()

                    self._grid_with_markers[tuple(self._last_target.idx)] += 1
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
        return gui.Widget.EventCallbackResult.IGNORED