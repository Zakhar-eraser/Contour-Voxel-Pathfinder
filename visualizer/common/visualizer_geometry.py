import numpy as np
import open3d as o3d

def create_mark(position, size, color):
    mark = o3d.geometry.TriangleMesh.create_sphere(size)
    mark.compute_vertex_normals()
    mark.translate(position)
    mark.paint_uniform_color(color)
    return mark

def make_route_geometry(route, line_width, marker_size):
    line_material = o3d.visualization.rendering.MaterialRecord()
    line_material.shader = "unlitLine"
    line_material.line_width = line_width
    marker_material = o3d.visualization.rendering.MaterialRecord()
    marker_material.shader = "defaultLit"
    line_sets = []
    spheres = []

    while route is not None:
        visit_lines = np.arange(len(route.visit_points) + 1)[:, np.newaxis]
        visit_lines = np.concatenate((visit_lines, visit_lines + 1), axis=1)
        visit_colors = [[1, 0, 0] for _ in range(len(visit_lines))]
        visit_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([route.point] + route.visit_points +
                [route.next_point.point]),
            lines=o3d.utility.Vector2iVector(visit_lines))
        visit_line_set.colors = o3d.utility.Vector3dVector(visit_colors)
        observe_len = len(route.observe_points)
        observe_lines = np.arange(observe_len)[:, np.newaxis]
        observe_lines = np.concatenate((observe_lines, np.full(observe_len, observe_len)), axis=1)
        observe_colors = [[0, 0, 1] for _ in range(len(observe_lines))]
        observe_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(route.observe_points + [route.next_point.point]),
            lines=o3d.utility.Vector2iVector(observe_lines))
        observe_line_set.colors = o3d.utility.Vector3dVector(observe_colors)
        line_sets += [visit_line_set, observe_line_set]
        spheres.append(create_mark(route.point, marker_size, [1, 0, 0]))
        for mark in route.observe_points:
            spheres.append(create_mark(mark, marker_size, [0, 0, 1]))
        
        route = route.next_point

    return [[line_sets, line_material], [spheres, marker_material]]
