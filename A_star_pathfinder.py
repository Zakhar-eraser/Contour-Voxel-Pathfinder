import numpy as np
from queue import PriorityQueue

voxel_step_cost = 1.0
obstacle_cost = 1.0

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

def count_obstacles(grid, idx):
    count = 0
    for x in range(-1, 2):
        for y in range(-1, 2):
            for z in range(-1, 2):
                if not index_in_bounds(idx + np.array((x, y, z), np.int32), grid):
                    count += 1
    return count

#def get_center_neighbours(grid, vox):
#    nghbrs = list()
#    for shift in range(3):
#        for i in (-1, 1):
#            new_idx = vox + np.roll(np.array((i, 0, 0), np.int32), shift)
#            if index_in_bounds(new_idx, grid):
#                nghbrs.append(VoxelNode(new_idx, vox, voxel_step_cost + count_obstacles(grid, new_idx) * obstacle_cost))
#    return nghbrs

def limit_max_idx(idx, shape):
    if idx[0] >= shape: idx[0] = shape - 1
    if idx[1] >= shape: idx[1] = shape - 1
    if idx[2] >= shape: idx[2] = shape - 1
    return idx

def limit_min_index(idx):
    if idx[0] < 0: idx[0] = 0
    if idx[1] < 0: idx[1] = 0
    if idx[2] < 0: idx[2] = 0
    return idx

def get_neighbours_occupancy(grid, vox):
    min_idx = limit_min_index(vox.idx - np.ones((1, 3), np.int32))
    max_idx = limit_max_idx(vox.idx + np.ones((1, 3), np.int32))
    neigh_grid = np.copy(grid[min_idx[0]:max_idx[0], min_idx[1]:max_idx[1], min_idx[2]:max_idx[2]])
    

#def get_neighbours(grid, vox):
    


def manh_dist(v1, v2):
    v = v1.idx - v2.idx
    return abs(v[0]) + abs(v[1]) + abs(v[2])

def sqr_dist(v1, v2):
    v = v1.idx - v2.idx
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]

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
            new_cost = cost_graph[current].cost + next.cost
            if (next not in cost_graph) or (new_cost < cost_graph[next].cost):
                next.cost = new_cost
                cost_graph[next] = next
                priority = new_cost + manh_dist(next, end_node)
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
   