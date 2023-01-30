import numpy as np
import open3d as o3d
from structures.destination_list import Transfer
from structures.destination_list import Targets
from structures.destination_list import WritableTargets

class MarkerColors:
    """Dictonary of colors"""
    VISIT = [1, 0, 0]
    START = [0, 1, 0]
    OBSERVE = [0, 0, 1]
    SELECTED = [0.62, 0.17, 0.41]
    TAKEOFF = [0.9, 0.5, 0.1]

    def get_color(transfer_type):
        if transfer_type == Transfer.VISIT:
            color = MarkerColors.VISIT
        elif transfer_type == Transfer.OBSERVE:
            color = MarkerColors.OBSERVE
        elif transfer_type == Transfer.START:
            color = MarkerColors.START
        elif transfer_type == Transfer.TAKEOFF:
            color = MarkerColors.TAKEOFF
        else:
            color = [0, 0, 0]
        return color

def create_mark(position, size, color):
    mark = o3d.geometry.TriangleMesh.create_sphere(size)
    mark.compute_vertex_normals()
    mark.translate(position)
    mark.paint_uniform_color(color)
    return mark

def create_line(start, end, color):
    geometry = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector((start, end)),
        lines=o3d.utility.Vector2iVector([[0, 1]]))
    geometry.colors = o3d.utility.Vector3dVector([color])
    return geometry

def make_route_lines(route, line_width, marker_size):
    line_material = o3d.visualization.rendering.MaterialRecord()
    line_material.shader = "unlitLine"
    line_material.line_width = line_width
    marker_material = o3d.visualization.rendering.MaterialRecord()
    marker_material.shader = "defaultLit"
    line_sets = []

    while route.next_point is not None:
        visit_lines = np.arange(len(route.visit_points) + 1)[:, np.newaxis]
        visit_lines = np.concatenate((visit_lines, visit_lines + 1), axis=1)
        visit_colors = [[1, 0, 0] for _ in range(len(visit_lines))]
        visit_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([route.point] + route.visit_points +
                [route.next_point.point]),
            lines=o3d.utility.Vector2iVector(visit_lines))
        visit_line_set.colors = o3d.utility.Vector3dVector(visit_colors)
        observe_len = len(route.observe_points)
        line_sets.append(visit_line_set)
        if observe_len:
            observe_lines = np.arange(observe_len)[:, np.newaxis]
            observe_lines = np.concatenate((observe_lines, np.full((observe_len, 1), observe_len)), axis=1)
            observe_colors = [[0, 0, 1] for _ in range(len(observe_lines))]
            observe_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(route.observe_points + [route.next_point.point]),
            lines=o3d.utility.Vector2iVector(observe_lines))
            observe_line_set.colors = o3d.utility.Vector3dVector(observe_colors)
            line_sets.append(observe_line_set)
        
        route = route.next_point

    return [line_sets, line_material]
    
def get_lines_from_targets(targets, line_width):
    line_material = o3d.visualization.rendering.MaterialRecord()
    line_material.shader = "unlitLine"
    line_material.line_width = line_width
    
def get_targets(writable_targets, marker_size):
    root = None
    if writable_targets is not None:
        root = Targets(
            create_mark(writable_targets.pos, marker_size, MarkerColors.get_color(writable_targets.transfer)),
            writable_targets.name, writable_targets.id)
        root.transfer = writable_targets.transfer
        target = root
        writable_targets = writable_targets.target
        while writable_targets is not None:
            target.add(Targets(create_mark(writable_targets.pos, marker_size, MarkerColors.get_color(writable_targets.transfer)),
                writable_targets.name, writable_targets.id), writable_targets.transfer)
            target = target.target
            writable_targets = writable_targets.target
    return root