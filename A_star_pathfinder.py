import numpy as np
from queue import PriorityQueue

voxel_step_cost = 1.0
obstacle_cost = 0.0

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

class CostGraph:
    """Counts and saves the cost of voxels"""

    def __init__(self, grid):
        self.graph = {}
        self.grid = grid
    
    def __getitem__(self, voxel):
        return self.graph[voxel]
         
    def __index_in_bounds__(self, idx):
        bounds = self.grid.shape
        return ((idx[0] < bounds[0]) and (idx[1] < bounds[1]) and (idx[2] < bounds[2])
            and not self.grid[idx[0], idx[1], idx[2]]) and ((idx[0] >= 0) and
            (idx[1] >= 0) and (idx[2] >= 0))

    def __local_free_grid__(self, idx):
        local_grid = np.zeros((3, 3, 3), np.int32)
        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(-1, 2):
                    if self.__index_in_bounds__(idx + np.array((x, y, z), np.int32)):
                        local_grid[x, y, z] = 1
        return local_grid

    def __get_neighbours_occupancy__(self, local_grid):
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

    def __get_neighbours__(self, vox):
        idx = vox.idx
        nbrs_grid = self.__get_neighbours_occupancy__(self.__local_free_grid__(idx))
        nbrs = list()
        for x in range(-1, 2):
            for y in range(-1, 2):
                for z in range(-1, 2):
                    local_idx = (x, y, z)
                    if nbrs_grid[local_idx]:
                        nbrs.append(np.array(local_idx) + idx)
        return nbrs

    def __cost__(self, voxel):
        #return (26 - len(voxel.nbrs)) * obstacle_cost
        return obstacle_cost if len(voxel.nbrs) < 26 else 0

    def __setitem__(self, key, voxel):
        voxel.nbrs = self.__get_neighbours__(voxel)
        voxel.cost = self.__cost__(voxel)
        self.graph[key] = voxel

def get_neighbours(voxel):
    nbrs = list()
    for idx in voxel.nbrs:
        nbrs.append(VoxelNode(idx, voxel))
    return nbrs

def manh_dist(v1, v2):
    v = v1.idx - v2.idx
    return abs(v[0]) + abs(v[1]) + abs(v[2])

def sqr_dist(v1, v2):
    v = v1.idx - v2.idx
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]

def heuristics(cur, next, end):
    return voxel_step_cost * manh_dist(cur, next) + manh_dist(next, end)

def find_path_A_star(grid, start, end, stop_condition):
    frontier = PriorityQueue()
    start_node = VoxelNode(start, None)
    end_node = VoxelNode(end, None)
    frontier.put((0, start_node))
    costs = CostGraph(grid)
    costs[start_node] = start_node

    while not frontier.empty():
        current = frontier.get()[1]

        if stop_condition(current, end_node):
            break
      
        for next in get_neighbours(current):
            new_cost = costs[current].cost + next.cost
            if (next not in costs.graph) or (new_cost < costs[next].cost):
                next.cost = new_cost
                costs[next] = next
                priority = new_cost + heuristics(current, next, end_node)
                frontier.put((priority, next))
    
    return costs.graph

def get_route_idx(graph, end):
    cur = graph[VoxelNode(end, None)]
    route = list()
    while cur.parent is not None:
        route.append(cur.idx)
        cur = cur.parent
    route.append(cur.idx)
    return np.array(route, dtype=np.int32)
   