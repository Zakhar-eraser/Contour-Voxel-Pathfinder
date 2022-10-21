import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import open3d as o3d

# This example displays a point cloud and if you Ctrl-click on a point
# (Cmd-click on macOS) it will show the coordinates of the point.
# This example illustrates:
# - custom mouse handling on SceneWidget
# - getting a the depth value of a point (OpenGL depth)
# - converting from a window point + OpenGL depth to world coordinate
class PointsSelectorApp:

    voxel_size = 1

    def __init__(self, cloud):
        # We will create a SceneWidget that fills the entire window, and then
        # a label in the lower left on top of the SceneWidget to display the
        # coordinate.
        app = gui.Application.instance
        self.window = app.create_window("Select points", 1024, 768)
        # Since we want the label on top of the scene, we cannot use a layout,
        # so we need to manually layout the window's children.
        self.window.set_on_layout(self._on_layout)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)
        self.info = gui.Label("Select target position")
        self.info.visible = True
        self.window.add_child(self.info)

        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)

        mat = rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        # Point size is in native pixels, but "pixel" means different things to
        # different platforms (macOS, in particular), so multiply by Window scale
        # factor.
        mat.point_size = 3 * self.window.scaling
        self.widget3d.scene.add_geometry("Point Cloud", cloud, mat)

        bounds = self.widget3d.scene.bounding_box
        center = bounds.get_center()
        self.widget3d.setup_camera(60, bounds, center)

        self.widget3d.set_on_mouse(self._on_mouse_widget3d)
        self.widget3d.set_on_key(self._on_keyboard_widget3d)
        self.boxes = list()
        self.mission_points = list()
        self.lock_geom = False

    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget3d.frame = r
        pref = self.info.calc_preferred_size(layout_context,
                                             gui.Widget.Constraints())
        self.info.frame = gui.Rect(r.x,
                                   r.get_bottom() - pref.height, pref.width,
                                   pref.height)

    def _on_keyboard_widget3d(self, event):
        if event.type == gui.KeyEvent.Type.DOWN and self.lock_geom and len(self.boxes):
            vs = PointsSelectorApp.voxel_size
            mat = rendering.MaterialRecord()
            mat.shader = "defaultLit"
            if len(self.boxes) == 1:
                name = "target"
                box = self.boxes[0]
            else:
                name = "start"
                box = self.boxes[1]
            
            def move_box():
                if event.key == gui.KeyName.W:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((0, 0, vs))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.S:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((0, 0, -vs))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.LEFT:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((-vs, 0, 0))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.RIGHT:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((vs, 0, 0))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.UP:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((0, vs, 0))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.DOWN:
                    self.widget3d.scene.remove_geometry(name)
                    box.translate((0, -vs, 0))
                    self.widget3d.scene.add_geometry(name, box, mat)
                elif event.key == gui.KeyName.ENTER:
                    self.lock_geom = False
                    self.mission_points.append(box.get_center())
                    self.info.text = "Select start position"
                    self.window.set_needs_layout()
                else:
                    return gui.Widget.EventCallbackResult.IGNORED

            gui.Application.instance.post_to_main_thread(
                self.window, move_box)
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
                # Note that np.asarray() reverses the axes.
                depth = np.asarray(depth_image)[y, x]

                if depth == 1.0:  # clicked on nothing (i.e. the far plane)
                    return gui.Widget.EventCallbackResult.IGNORED
                else:
                    world = self.widget3d.scene.camera.unproject(
                        event.x, event.y, depth, self.widget3d.frame.width,
                        self.widget3d.frame.height)
                
                self.lock_geom = True

                def update_geometries():
                    vs = PointsSelectorApp.voxel_size
                    box = o3d.geometry.TriangleMesh.create_box(vs, vs, vs)
                    box.compute_vertex_normals()
                    box.translate(world - box.get_center())
                    mat = rendering.MaterialRecord()
                    mat.shader = "defaultLit"
                    if len(self.boxes) == 0:
                        box.paint_uniform_color((1, 0, 0))
                        name = "target"
                    else:
                        box.paint_uniform_color((0, 0, 1))
                        name = "start"
                    
                    self.info.text = "Move position"
                    self.window.set_needs_layout()
                    self.boxes.append(box)
                    self.widget3d.scene.add_geometry(name, box, mat)

                gui.Application.instance.post_to_main_thread(
                    self.window, update_geometries)

            self.widget3d.scene.scene.render_to_depth_image(depth_callback)
            return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED
    
    def get_mission_points(self):
        return self.mission_points