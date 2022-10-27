from shutil import move
import numpy as np
from queue import PriorityQueue
import voxel_raycast as vr

voxel_step_cost = 1.0
obstacle_cost = 10.0
obstacle_check_depth = 0
observation_range = 40

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
                    local_grid[tuple(n1 + n2 + 1)] or\
                    local_grid[tuple(n1 + n3 + 1)] or\
                    local_grid[tuple(n2 + n3 + 1)]:
                    local_grid[tuple(n1 + n2 + n3 + 1)] = 1
    return local_grid

def get_neighbours(grid, idx):
    local_grid = np.copy(
        grid[idx[0] - 1:idx[0] + 2, idx[1] - 1:idx[1] + 2, idx[2] - 1:idx[2] + 2])
    #print(local_grid)
    nbrs_grid = get_neighbours_occupancy(local_grid)
    nbrs = list()
    for x in range(3):
        for y in range(3):
            for z in range(3):
                local_idx = (x, y, z)
                if not nbrs_grid[local_idx]:
                    nbrs.append(np.array(local_idx) - 1 + idx)
    return nbrs

def sqr_dist(v1, v2):
    return np.sum(np.square(v1 - v2))

def manh_dist(v1, v2):
    return np.sum(np.abs(v1 - v2))

def heuristics(move_priority, cur, next, end):
    return manh_dist(cur, end) + move_priority(cur, next)

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

    return obstacle_check_depth

def plane_move_priority(cur, next):
    moves = np.abs(cur - next)
    dist_cost = 1.4 if moves[0] and moves[1] else 1 if moves[0] or moves[1] else 0
    if moves[2]: dist_cost += 2
    return dist_cost

def euclidian_priority(cur, next):
    dist_cost = manh_dist(cur, next)
    dist_cost = 1 if dist_cost == 1 else 1.4 if dist_cost == 2 else 1.7
    return dist_cost

def get_cost(grid, vox):
    obst_cost = obstacle_cost * (obstacle_check_depth -
        obstacle_closeness(grid, vox))

    cost = obst_cost + 1
    return cost

def same_point_cond(current, target, grid):
    return np.array_equal(current, target)

def visibility_cond(current, target, grid):
    return observation_range * observation_range > sqr_dist(current, target) and (
        not vr.check_intersection(current, target, grid))

def find_path_A_star(grid, start, end,
    stop_cond = same_point_cond,
    move_priority = euclidian_priority):
    frontier = PriorityQueue()
    frontier.put((0, Node(start, 0)))
    costs = {tuple(start): 0.0}
    parents = {tuple(start): None}

    while not frontier.empty():
        current = frontier.get()[1].idx

        if stop_cond(current, end, grid):
            if not np.array_equal(current, end):
                parents[tuple(end)] = current
            break
        
        nbrs = get_neighbours(grid, current)
        for next in nbrs:
            new_cost = costs[tuple(current)] + get_cost(grid, next)
            if (tuple(next) not in costs) or (new_cost < costs[tuple(next)]):
                parents[tuple(next)] = current
                costs[tuple(next)] = new_cost
                next_node = Node(next, new_cost + heuristics(move_priority, current, next, end))
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
   