import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d
import structures.destination_list as dl

class PointsSelectorApp:

    voxel_size = 1

    def __init__(self, cloud):
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
        self.widget3d.scene.add_geometry("Point Cloud", cloud, mat)

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
                vs = PointsSelectorApp.voxel_size
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
                            geometry.translate((0, 0, vs))
                        elif event.key == gui.KeyName.S:
                            geometry.translate((0, 0, -vs))
                        elif event.key == gui.KeyName.LEFT:
                            geometry.translate((-vs, 0, 0))
                        elif event.key == gui.KeyName.RIGHT:
                            geometry.translate((vs, 0, 0))
                        elif event.key == gui.KeyName.UP:
                            geometry.translate((0, vs, 0))
                        elif event.key == gui.KeyName.DOWN:
                            geometry.translate((0, -vs, 0))
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
                gui.KeyModifier.CTRL):

            if self.lock_geom:
                return gui.Widget.EventCallbackResult.IGNORED

            def depth_callback(depth_image):
                x = event.x - self.widget3d.frame.x
                y = event.y - self.widget3d.frame.y
                depth = np.asarray(depth_image)[y, x]

                if depth == 1.0:
                    return gui.Widget.EventCallbackResult.IGNORED
                else:
                    world = self.widget3d.scene.camera.unproject(
                        event.x, event.y, depth, self.widget3d.frame.width,
                        self.widget3d.frame.height)               

                def update_geometries():
                    vs = PointsSelectorApp.voxel_size
                    mark = o3d.geometry.TriangleMesh.create_sphere(vs / 2)
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

            self.widget3d.scene.scene.render_to_depth_image(depth_callback)
            return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED
