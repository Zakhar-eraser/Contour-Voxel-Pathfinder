import numpy as np

class PassNode:
    """An object of the class contains voxel`s indexes and cost of the voxel"""

    def __init__(self, idx, parent, cost):
        self.idx = idx
        self.cost = cost
        self.
    
    def __eq__(self, __o: object) -> bool:
        return np.array_equal(self.idx, __o.idx)
    
    def __hash__(self) -> int:
        return hash(tuple(self.idx))

    def __lt__(self, other):
        return self.cost < other.cost

class RouteNode:
    """Graph of """

def occupancy_grid_to_nav_graph(occupancy_grid):
