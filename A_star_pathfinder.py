from math import dist
import numpy as np
from queue import PriorityQueue

class VoxelNode:
    """An object of the class contains voxel`s indexes and cost of movement to the voxel"""

    def __init__(self, idx, parent, cost):
        self.idx = idx
        self.parent = parent
        self.cost = cost
    
    def __eq__(self, __o: object) -> bool:
        return np.array_equal(self.idx, __o.idx)
    
    def __hash__(self) -> int:
        return hash(tuple(self.idx))

    def __lt__(self, other):
        return self.cost < other.cost

    def get_neigbours(self):
        fwd = VoxelNode(self.idx + np.array([1, 0, 0]), self, 1)
        bwd = VoxelNode(self.idx + np.array([-1, 0, 0]), self,1)
        rgt = VoxelNode(self.idx + np.array([0, 1, 0]), self, 1)
        lft = VoxelNode(self.idx + np.array([0, -1, 0]), self, 1)
        up = VoxelNode(self.idx + np.array([0, 0, 1]), self, 1)
        dwn = VoxelNode(self.idx + np.array([0, 0, -1]), self, 1)
        fwd_up = VoxelNode(self.idx + np.array([1, 0, 1]), self, 2)
        fwd_dwn = VoxelNode(self.idx + np.array([1, 0, -1]), self, 2)
        bwd_up = VoxelNode(self.idx + np.array([-1, 0, 1]), self, 2)
        bwd_dwn = VoxelNode(self.idx + np.array([-1, 0, -1]), self, 2)
        fwd_rgt = VoxelNode(self.idx + np.array([1, 1, 0]), self, 2)
        fwd_lft = VoxelNode(self.idx + np.array([1, -1, 0]), self, 2)
        bwd_rgt = VoxelNode(self.idx + np.array([-1, 1, 0]), self, 2)
        bwd_lft = VoxelNode(self.idx + np.array([-1, -1, 0]), self, 2)
        rgt_up = VoxelNode(self.idx + np.array([0, 1, 1]), self, 2)
        rgt_dwn = VoxelNode(self.idx + np.array([0, 1, -1]), self, 2)
        lft_up = VoxelNode(self.idx + np.array([0, -1, 1]), self, 2)
        lft_dwn = VoxelNode(self.idx + np.array([0, -1, -1]), self, 2)
        fwd_rgt_up = VoxelNode(self.idx + np.array([1, 1, 1]), self, 3)
        bwd_rgt_up = VoxelNode(self.idx + np.array([-1, 1, 1]), self, 3)
        fwd_lft_up = VoxelNode(self.idx + np.array([1, -1, 1]), self, 3)
        bwd_lft_up = VoxelNode(self.idx + np.array([-1, -1, 1]), self, 3)
        fwd_rgt_dwn = VoxelNode(self.idx + np.array([1, 1, -1]), self, 3)
        bwd_rgt_dwn = VoxelNode(self.idx + np.array([-1, 1, -1]), self, 3)
        fwd_lft_dwn = VoxelNode(self.idx + np.array([1, -1, -1]), self, 3)
        bwd_lft_dwn = VoxelNode(self.idx + np.array([-1, -1, -1]), self, 3)
        return [fwd, bwd, rgt, lft, up, dwn, fwd_rgt_up, fwd_rgt_dwn,
            fwd_up, fwd_dwn, bwd_up, bwd_dwn, fwd_rgt, fwd_lft, bwd_rgt, bwd_lft,
            rgt_up, rgt_dwn, lft_up, lft_dwn,
            bwd_rgt_dwn, bwd_rgt_up, fwd_lft_dwn, fwd_lft_up, bwd_lft_dwn, bwd_lft_up]
    
def index_in_bounds(idx, grid):
    bounds = grid.shape
    return ((idx[0] < bounds[0]) and (idx[1] < bounds[1]) and (idx[2] < bounds[2])
        and not grid[idx[0], idx[1], idx[2]]) and ((idx[0] >= 0) and
        (idx[1] >= 0) and (idx[2] >= 0))

def manh_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2])

def sqr_dist(p1, p2):
    p = p1 - p2
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2]

def find_path_A_star(grid, start, end, stop_condition):
    frontier = PriorityQueue()
    start_node = VoxelNode(start, None, 0.0)
    end_node = VoxelNode(end, None, None)
    frontier.put((0, start_node))
    cost_graph = {start_node: start_node}

    while not frontier.empty():
        current = frontier.get()[1]

        if stop_condition(current, end_node):
            break
      
        for next in current.get_neigbours():
            if index_in_bounds(next.idx, grid):
                new_cost = cost_graph[current].cost + next.cost
                if (next not in cost_graph) or (new_cost < cost_graph[next].cost):
                    next.cost = new_cost
                    cost_graph[next] = next
                    priority = new_cost + sqr_dist(next.idx, end_node.idx)
                    frontier.put((priority, next))
    
    return cost_graph

def get_route_idx(graph, end):
    cur = graph[VoxelNode(end, None, None)]
    route = list()
    while cur.parent is not None:
        route.append(cur.idx)
        cur = cur.parent
    route.append(cur.idx)
    return np.array(route, dtype=np.int32)
   