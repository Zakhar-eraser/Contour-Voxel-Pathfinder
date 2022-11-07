import numpy as np
from queue import PriorityQueue
import utils.voxel_raycast as vr
import utils.neighbours as nb
import structures.priority_node as pn

height_step_priority = 1
obstacle_cost = 10.0
obstacle_check_depth = 0
observation_range = 40

def sqr_dist(v1, v2):
    return np.sum(np.square(v1 - v2))

def manh_dist(v1, v2):
    return np.sum(np.abs(v1 - v2))

def heuristics(move_priority, cur, next, end):
    return manh_dist(next, end) + move_priority(cur, next)

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
    return moves[2] * height_step_priority

def get_cost(grid, vox):
    obst_cost = obstacle_cost * (obstacle_check_depth -
        obstacle_closeness(grid, vox))

    cost = obst_cost + 1
    return cost

def same_point_cond(current, target, grid):
    return np.array_equal(current, target)

def visibility_cond(current, target, grid):
    return observation_range * observation_range > sqr_dist(current, target) and (
        vr.check_intersection(current, target, grid) is None)

def find_path_A_star(grid, start, end,
    stop_cond = same_point_cond,
    move_priority = lambda c, n: 0):
    frontier = PriorityQueue()
    frontier.put((0, pn.Node(start, 0)))
    costs = {tuple(start): 0.0}
    parents = {tuple(start): None}

    while not frontier.empty():
        current = frontier.get()[1].idx

        if stop_cond(current, end, grid):
            if not np.array_equal(current, end):
                parents[tuple(end)] = current
            break
        
        nbrs = nb.get_neighbours(grid, current)
        for next in nbrs:
            new_cost = costs[tuple(current)] + get_cost(grid, next)
            if (tuple(next) not in costs) or (new_cost < costs[tuple(next)]):
                parents[tuple(next)] = current
                costs[tuple(next)] = new_cost
                next_node = pn.Node(next, new_cost +
                    heuristics(move_priority, current, next, end))
                frontier.put((next_node.priority, next_node))
    
    return parents

def get_route(graph, min_bound, end):
    cur = end
    route = list()
    while graph[tuple(cur)] is not None:
        route.append(cur)
        cur = graph[tuple(cur)]
    route.append(cur)
    return np.array(route) + min_bound  # Use min_voxel instead point
   