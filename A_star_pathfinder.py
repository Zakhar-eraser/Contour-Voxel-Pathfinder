import numpy as np
from queue import PriorityQueue

voxel_step_cost = 1.0
obstacle_cost = 0.0
obstacle_check_depth = 0

class Node:
    """Interface to work numpy array with PriorityQueue"""

    def __init__(self, idx, priority):
        self.idx = idx
        self.priority = priority

    def __eq__(self, other):
        return self.priority == other.priority
    
    def __hash__(self):
        return hash(tuple(self.idx))

    def __lt__(self, other):
        return self.priority < other.priority

def get_neighbours_occupancy(local_grid):
    local_grid[1, 1, 1] = 1
    for order in range(3):
        for i in (-1, 1):
            for j in (-1, 1):
                n1 = np.roll((i, 0, 0), order)
                n2 = np.roll((0, j, 0), order)
                if local_grid[tuple(n1 + 1)] or local_grid[tuple(n2 + 1)]:
                    local_grid[tuple(n1 + n2 + 1)] = 1

    for i in (-1, 1):
        for j in (-1, 1):
            for k in (-1, 1):
                n1 = np.array((i, 0, 0), np.int8)
                n2 = np.array((0, j, 0), np.int8)
                n3 = np.array((0, 0, k), np.int8)
                if local_grid[tuple(n1 + 1)] or local_grid[tuple(n2 + 1)] or\
                    local_grid[tuple(n3  + 1)] or\
                    local_grid[tuple(np.array((i, j, 0), np.int8) + 1)] or\
                    local_grid[tuple(np.array((i, 0, k), np.int8) + 1)] or\
                    local_grid[tuple(np.array((0, j, k), np.int8) + 1)]:
                    local_grid[tuple(n1 + n2 + n3 + 1)] = 1
    #print(local_grid)
    return local_grid

def get_neighbours(grid, idx):
    nbrs_grid = get_neighbours_occupancy(np.copy(
        grid[idx[0] - 1:idx[0] + 2, idx[1] - 1:idx[1] + 2, idx[2] - 1:idx[2] + 2]))
    nbrs = list()
    for x in range(-1, 2):
        for y in range(-1, 2):
            for z in range(-1, 2):
                local_idx = (x, y, z)
                if not nbrs_grid[local_idx]:
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
            for i in range(-depth + 1, depth):
                for j in range(-depth + 1, depth):
                    for k in (-depth, depth):
                        if grid[tuple(idx + np.roll((i, j, k), shift))]:
                            return depth

            for i in (-depth, depth):
                for j in range(-depth + 1, depth):
                    for k in (-depth, depth):
                        if grid[tuple(idx + np.roll((i, j, k), shift))]:
                            return depth
            
            for i in (-depth, depth):
                for j in (-depth, depth):
                    for k in (-depth, depth):
                        if grid[tuple(idx + np.roll((i, j, k), shift))]:
                            return depth

    return 100

def get_cost(grid, current, next):
    dist_cost = manh_dist(current, next)
    dist_cost = 1 if dist_cost == 1 else 1.4 if dist_cost == 2 else 1.7

    obst_cost = obstacle_cost / obstacle_closeness(grid, next)

    cost = dist_cost + obst_cost
    return cost

def find_path_A_star(grid, start, end, stop_condition):
    frontier = PriorityQueue()
    frontier.put((0, Node(start, 0)))
    costs = {tuple(start): 0.0}
    parents = {tuple(start): None}

    while not frontier.empty():
        current = frontier.get()[1].idx

        if stop_condition(current, end):
            break
        
        nbrs = get_neighbours(grid, current)
        for next in nbrs:
            new_cost = costs[tuple(current)] + get_cost(grid, current, next)
            if (tuple(next) not in costs) or (new_cost < costs[tuple(next)]):
                parents[tuple(next)] = current
                costs[tuple(next)] = new_cost
                next_node = Node(next, new_cost + heuristics(next, end))
                frontier.put((next_node.priority, next_node))
    
    return parents

def get_route_idx(graph, end):
    cur = end
    route = list()
    while graph[tuple(cur)] is not None:
        route.append(cur)
        cur = graph[tuple(cur)]
    route.append(cur)
    return np.array(route, dtype=np.int32) - 1
   