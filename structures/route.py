from utils.grids.occupancy_grid import idx2pos
from utils.grids.occupancy_grid import vect_idx2pos
from structures.destination_list import Transfer

class Route:
    """List with array of points to destinate next point as value"""
    
    def __init__(self):
        self.next_point = None
        self.point = None
        self.visit_points = []
        self.observe_points = []

def route_idx2pos(min_bound, route_idx, voxel_size):
    route = Route()
    proute = route
    while route_idx is not None:
        proute.point = idx2pos(min_bound, route_idx.point, voxel_size)
        proute.visit_points = vect_idx2pos(min_bound, route_idx.visit_points, voxel_size)
        proute.observe_points = vect_idx2pos(min_bound, route_idx.observe_points, voxel_size)
        route_idx = route_idx.next_point
        if route_idx.next_point is not None:
            proute.next_point = Route()
            proute = proute.next_point
    
    return route

def route2list(route):
    points = []
    while route is not None:
        points.append(route.point)
        points += route.visit_points
        route = route.next_point
    
    return points

def targets2route(targets):
    route_root = Route()
    route = route_root
    while targets is not None:
        point = targets.mark.get_center()
        if targets.transfer == Transfer.DESTINATE or targets.transfer is None:
            route.point = point
        else:
            route.observe_points.append(point)
        targets = targets.target
        if targets is not None:
            route.next_point = Route()
            route = route.next_point
    
    return route_root