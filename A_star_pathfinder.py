import numpy as np
from queue import PriorityQueue

voxel_step_cost = 1.0
obstacle_cost = 0.0
obstacle_check_depth = 2

class VoxelNode:
    """An object of the class contains voxel`s indexes and cost of movement to the voxel"""

    def __init__(self, idx, parent):
        self.idx = idx
        self.parent = parent
        self.cost = 0.0
        self.nbrs = list()
    
    def __eq__(self, __o: object) -> bool:
        return np.array_equal(self.idx, __o.idx)
    
    def __hash__(self) -> int:
        return hash(tuple(self.idx))

    def __lt__(self, other):
        return self.cost < other.cost

def index_in_bounds(grid, idx):
    bounds = grid.shape
    return ((idx[0] < bounds[0]) and (idx[1] < bounds[1]) and (idx[2] < bounds[2])  and (
        (idx[0] >= 0) and (idx[1] >= 0) and (idx[2] >= 0))
        and not grid[idx[0], idx[1], idx[2]])

def local_free_grid(grid, idx):
    local_grid = np.zeros((3, 3, 3), np.int32)
    for x in range(-1, 2):
        for y in range(-1, 2):
            for z in range(-1, 2):
                if index_in_bounds(grid, idx + np.array((x, y, z), np.int32)):
                    local_grid[x, y, z] = 1
    return local_grid

def get_neighbours_occupancy(local_grid):
    for order in range(3):
        for i in (-1, 1):
            for j in (-1, 1):
                n1 = np.roll(np.array((i, 0, 0), np.int32), order)
                n2 = np.roll(np.array((0, j, 0), np.int32), order)
                if (not local_grid[tuple(n1)]) or (not local_grid[tuple(n2)]):
                    local_grid[tuple(n1 + n2)] = 0

    for i in (-1, 1):
        for j in (-1, 1):
            for k in (-1, 1):
                n1 = np.array((i, 0, 0), np.int32)
                n2 = np.array((0, j, 0), np.int32)
                n3 = np.array((0, 0, k), np.int32)
                if (not local_grid[tuple(n1)]) or (not local_grid[tuple(n2)]) or (
                    not local_grid[tuple(n3)]
                    ):
                    local_grid[tuple(n1 + n2 + n3)] = 0
    return local_grid

def get_neighbours(grid, idx):
    nbrs_grid = get_neighbours_occupancy(local_free_grid(grid, idx))
    nbrs = list()
    for x in range(-1, 2):
        for y in range(-1, 2):
            for z in range(-1, 2):
                local_idx = (x, y, z)
                if nbrs_grid[local_idx]:
                    nbrs.append(np.array(local_idx) + idx)
    return nbrs

def manh_dist(v1, v2):
    v = v1 - v2
    return abs(v[0]) + abs(v[1]) + abs(v[2])

def sqr_dist(v1, v2):
    v = v1 - v2
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]

def heuristics(cur, end):
    return manh_dist(cur, end)

def obstacle_closeness(grid, idx):
    for depth in range(1, obstacle_check_depth + 1):
        for shift in range(0, 3):
            for i in range(-depth + 1, depth - 1):
                for j in range(-depth + 1, depth - 1):
                    for k in (-depth, depth):
                        if index_in_bounds(grid)

def get_cost(grid, idx):


def find_path_A_star(grid, start, end, stop_condition):
    frontier = PriorityQueue()
    frontier.put((0, start))
    costs = {start: 0.0}
    parents = {start: None}

    while not frontier.empty():
        current = frontier.get()[1]

        if stop_condition(current, end):
            break
      
        for next in get_neighbours(current):
            new_cost = costs[current] + get_cost(next)
            if (next not in costs) or (new_cost < costs[next]):
                costs[next] = new_cost
                priority = new_cost + heuristics(next, end)
                frontier.put((priority, next))
    
    return costs

def get_route_idx(graph, end):
    cur = end
    route = list()
    while graph[cur] is not None:
        route.append(cur)
        cur = graph[cur]
    route.append(cur)
    return np.array(route, dtype=np.int32)
   