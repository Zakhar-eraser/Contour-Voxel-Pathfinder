import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
from structures.destination_list import Path
from structures.destination_list import Transfer
import utils.voxel_raycast as vr
from utils.grids.occupancy_grid import pos2idx
from utils.grids.occupancy_grid import idx2pos
from utils.geometry.line_plane_intersection import find_intersection
from utils.geometry.distances import sqr_dist

class PointsSelectorApp:

    def __init__(self, voxel_grid, occupancy_grid, targets = None):
        vx = voxel_grid
        self._grid = occupancy_grid
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
        self._transfer_type = Transfer.DESTINATE
        self._lock_geom = False
        self._dest_count = 0
        self._obs_count = 0

        self._vs = vx.voxel_size
        self._bounding_box = vx.get_axis_aligned_bounding_box()
        self._min_bound = self._bounding_box.min_bound
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
            self._transfer_type = Transfer.DESTINATE
    
    def _on_calc_clicked(self):
        gui.Application.instance.quit()

    def _on_keyboard_widget3d(self, event):
        if event.type == gui.KeyEvent.Type.DOWN:
            if self._lock_geom:
                mat = rendering.MaterialRecord()
                name = self._last_target.name
                if event.key == gui.KeyName.ENTER:
                    self._lock_geom = False
                    self._info.text = "Adding new target"
                    if self._last_target.origin is not None:
                        mat.shader = "unlitLine"
                        mat.line_width = 5
                        geometry = o3d.geometry.LineSet(
                            points=o3d.utility.Vector3dVector(
                                (self._last_target.mark.get_center(),
                                 self._last_target.origin.mark.get_center())),
                            lines=o3d.utility.Vector2iVector([[0, 1]]))
                        geometry.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
                        name += "_line"
                    else:
                        geometry = self.targets.mark
                        name = self.targets.name
                else:
                    mat.shader = "defaultLit"
                    geometry = self._last_target.mark
                    name = self._last_target.name
                    if event.key == gui.KeyName.W:
                        geometry.translate((0, 0, self._vs))
                    elif event.key == gui.KeyName.S:
                        geometry.translate((0, 0, -self._vs))
                    elif event.key == gui.KeyName.LEFT:
                        geometry.translate((-self._vs, 0, 0))
                    elif event.key == gui.KeyName.RIGHT:
                        geometry.translate((self._vs, 0, 0))
                    elif event.key == gui.KeyName.UP:
                        geometry.translate((0, self._vs, 0))
                    elif event.key == gui.KeyName.DOWN:
                        geometry.translate((0, -self._vs, 0))
                    else:
                        return gui.Widget.EventCallbackResult.IGNORED
                    self._widget3d.scene.remove_geometry(name)
                
                self._widget3d.scene.add_geometry(name, geometry, mat)
                return gui.Widget.EventCallbackResult.HANDLED
            else:
                if event.key == gui.BACKSPACE and self._last_target is not None: ## Возможность удалять старт
                    self._widget3d.scene.remove_geometry(self._last_target.name + "_line")
                    self._widget3d.scene.remove_geometry(self._last_target.name)
                    self._last_target = self._last_target.origin
                    if self._last_target is not None: self._last_target.target = None ## Изменено вместе с верхним
                else:
                    return gui.Widget.EventCallbackResult.IGNORED 
                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED


    def _on_mouse_widget3d(self, event):
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(
                gui.KeyModifier.CTRL) and (not self._lock_geom):
            cam_pos = self._widget3d.scene.camera.unproject(
                event.x, event.y, 0, self._widget3d.frame.width,
                self._widget3d.frame.height)
            cam_vox = pos2idx(self._min_bound, cam_pos, self._vs)
            dist_pos = self._widget3d.scene.camera.unproject(
                event.x, event.y, 0.99999, self._widget3d.frame.width,
                self._widget3d.frame.height)
            dist_vox = pos2idx(self._min_bound, dist_pos, self._vs)
            bnd_vox = self._get_nearest_bounding_box_voxel((cam_vox, dist_vox))
            target_vox = vr.check_intersection(bnd_vox, dist_vox ,self._grid)
            if target_vox is not None:
                world = idx2pos(self._min_bound, target_vox, self._vs)

                def update_geometries():
                    mark = o3d.geometry.TriangleMesh.create_sphere(self._vs / 2 + 0.1)
                    mark.compute_vertex_normals()
                    mark.translate(world)
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"

                    if self.targets is None:
                        mark.paint_uniform_color((0, 1, 0))
                        name = "start"
                        self._last_target = self.targets = Path(mark, name)
                    else:
                        if self._transfer_type == Transfer.DESTINATE:
                            mark_color = (1, 0, 0)
                            name = "dest_" + str(self._dest_count)
                            self._dest_count += 1
                        else:
                            mark_color = (0, 0, 1)
                            name = "obs_" + str(self._obs_count)
                            self._obs_count += 1
                        
                        mark.paint_uniform_color(mark_color)
                        self._last_target = self._last_target.add(Path(mark, name), self._transfer_type)
                    
                    self._info.text = "Move position"
                    self._window.set_needs_layout()

                    self._widget3d.scene.add_geometry(name, mark, mat)

                gui.Application.instance.post_to_main_thread(
                    self._window, update_geometries)

                self._lock_geom = True

                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED
