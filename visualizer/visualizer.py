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

class PointsSelectorApp:

    max_selecton_vox_dist = 100

    def __init__(self, voxel_grid, occupancy_grid):
        vx = voxel_grid
        self._grid = occupancy_grid

        app = gui.Application.instance
        self.window = app.create_window("Select points", 1024, 640)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        self.info = gui.Label("Select start position")
        self.info.visible = True
        self.window.add_child(self.info)

        # Setting gui`s panel
        em = self.window.theme.font_size
        separation_height = int(round(0.5 * em))
        self._settings_panel = gui.Vert(
            0, gui.Margins(0.25 * em, 0.25 * em, 0.25 * em, 0.25 * em))
        
        # adding checkbox on gui`s panel
        self._checkbox = gui.Checkbox("Stop when observable")
        self._checkbox.set_on_checked(self._on_checkbox)
        self._settings_panel.add_fixed(separation_height)
        self._settings_panel.add_child(self._checkbox)
        self.window.add_child(self._settings_panel)
        self.window.set_on_layout(self._on_layout)

        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene.set_background((0.8, 0.6, 0.6, 1))

        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"

        mat.point_size = 6 * self.window.scaling
        self.widget3d.scene.add_geometry("Point Cloud", vx, mat)

        bounds = self.widget3d.scene.bounding_box
        center = bounds.get_center()
        self.widget3d.setup_camera(60, bounds, center)

        self.widget3d.set_on_mouse(self._on_mouse_widget3d)
        self.widget3d.set_on_key(self._on_keyboard_widget3d)
        self.targets = None
        self.last_target = None
        self.transfer_type = dl.Transfer.DESTINATE
        self.lock_geom = False
        self.dest_count = 0
        self.obs_count = 0

        self._vs = vx.voxel_size
        self._bounding_box = vx.get_axis_aligned_bounding_box()
        self._min_bound = self._bounding_box.min_bound
        self._shape = (self._bounding_box.get_extent() /
            self._vs).astype('int32')
    
    def _idx_in_bounds(self, idx, min, max):
        in_bounds = False
        if (idx[0] > min[0]) and (idx[1] > min[1]) and (idx[2] > min[2]) and (
            (idx[0] <= max[0]) and (idx[1] <= max[1]) and (idx[2] <= max[2])):
            in_bounds = True
        return in_bounds
    
    def _get_nearest_bounding_box_voxel(self, line):
        s = self._shape
        idx = pos2idx(self._min_bound, line[0], self._vs)
        if (self._idx_in_bounds(idx, (0, 0, 0), s)):
            voxel = idx
        else:
            planes = np.array((((1, 1, 1), (1, 1, s[2]), (1, s[1], s[2])),
                      ((1, 1, 1), (s[0], 1, 1),(s[0], 1, s[2])),
                      ((1, 1, 1), (1, s[1], 1), (s[0], s[1], 1)),
                      ((s[0], 1, 1), (s[0], s[1], 1), s),
                      ((1, s[1], 1), (s[0], s[1], 1), s),
                      ((1, 1, s[2]), (s[0], 1, s[2]), s)))
            
            min_dist = (self.max_selecton_vox_dist *
                self.max_selecton_vox_dist)
            voxel = None
            for p in planes:
                target = find_intersection(line, p)
                if (target is not None) and (
                    self._idx_in_bounds(target, p[0], p[2])):
                    dist = sqr_dist(target, line[0])
                    if (dist < min_dist):
                        min_dist = dist
                        voxel = target
        return voxel
    
    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget3d.frame = r
        pref = self.info.calc_preferred_size(layout_context,
                                             gui.Widget.Constraints())
        
        self.info.frame = gui.Rect(r.x,
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
            self.transfer_type = dl.Transfer.OBSERVE
        else:
            self.transfer_type = dl.Transfer.DESTINATE

    def _on_keyboard_widget3d(self, event):
        if event.type == gui.KeyEvent.Type.DOWN:
            if self.lock_geom:
                mat = rendering.MaterialRecord()
                mat.shader = "defaultLit"

                def update_gui():
                    name = self.last_target.name
                    if event.key == gui.KeyName.ENTER:
                        self.lock_geom = False
                        self.info.text = "Adding new target"
                        if self.last_target.origin is not None:
                            geometry = o3d.geometry.LineSet(
                                points=o3d.utility.Vector3dVector(
                                    (self.last_target.mark.get_center(),
                                     self.last_target.origin.mark.get_center())),
                                lines=o3d.utility.Vector2iVector([[0, 1]]))
                            geometry.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
                            name += "_line"
                        else:
                            geometry = self.targets.mark
                            name = self.targets.name
                    else:
                        geometry = self.last_target.mark
                        name = self.last_target.name
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
                        self.widget3d.scene.remove_geometry(name)
                    self.widget3d.scene.add_geometry(name, geometry, mat)

                gui.Application.instance.post_to_main_thread(
                    self.window, update_gui)
                return gui.Widget.EventCallbackResult.HANDLED
            else:
                def update_gui():
                    if event.key == gui.BACKSPACE and self.last_target.origin is not None:
                        self.widget3d.scene.remove_geometry(self.last_target.name + "_line")
                        self.widget3d.scene.remove_geometry(self.last_target.name)
                        self.last_target = self.last_target.origin
                    else:
                        return gui.Widget.EventCallbackResult.IGNORED 
                gui.Application.instance.post_to_main_thread(
                    self.window, update_gui)
                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED


    def _on_mouse_widget3d(self, event):
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(
                gui.KeyModifier.CTRL) and (not self.lock_geom):

            depth = self.max_selecton_vox_dist * self._vs
            cam_pos = self.widget3d.scene.camera.unproject(
                event.x, event.y, 0, self.widget3d.frame.width,
                self.widget3d.frame.height)
            dist_pos = self.widget3d.scene.camera.unproject(
                event.x, event.y, 0.9995, self.widget3d.frame.width,
                self.widget3d.frame.height)
            bnd_vox = self._get_nearest_bounding_box_voxel((cam_pos, dist_pos))
            target_vox = vr.check_intersection(bnd_vox,
                pos2idx(self._min_bound, dist_pos, self._vs),
                self._grid)
            if target_vox is not None:
                world = idx2pos(self._min_bound, target_vox[0], self._vs)

                def update_geometries():
                    mark = o3d.geometry.TriangleMesh.create_sphere(self._vs / 2 + 0.1)
                    mark.compute_vertex_normals()
                    mark.translate(world)
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"

                    if self.targets is None:
                        mark.paint_uniform_color((0, 1, 0))
                        name = "start"
                        self.last_target = self.targets = dl.Path(mark, name)
                    else:
                        if self.transfer_type == dl.Transfer.DESTINATE:
                            mark_color = (1, 0, 0)
                            name = "dest_" + str(self.dest_count)
                            self.dest_count += 1
                        else:
                            mark_color = (0, 0, 1)
                            name = "obs_" + str(self.obs_count)
                            self.obs_count += 1
                        
                        mark.paint_uniform_color(mark_color)
                        self.last_target = self.last_target.add(dl.Path(mark, name), self.transfer_type)
                    
                    self.info.text = "Move position"
                    self.window.set_needs_layout()

                    self.widget3d.scene.add_geometry(name, mark, mat)

                gui.Application.instance.post_to_main_thread(
                    self.window, update_geometries)

                self.lock_geom = True

                return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED
