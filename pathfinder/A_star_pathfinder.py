import numpy as np
from queue import PriorityQueue
import utils.voxel_raycast as vr
import utils.neighbours as nb
import structures.priority_node as pn
from utils.geometry.distances import manh_dist
from utils.geometry.distances import sqr_dist

height_step_priority = 1
obstacle_cost = 10.0
obstacle_check_depth = 0
observation_range = 50

D1 = 10
D2 = 14
D3 = 17
H = 30

def heuristics(next, end):
    d = np.abs(next - end)
    dmin = min(d[0], d[1], d[2] * H)
    dmax = max(d[0], d[1], d[2] * H)
    dmid = d[0] + d[1] + d[2] - dmin - dmax
    return (D3 - D2) * dmin + (D2 - D1) * dmid + D1 * dmax

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
        obstacle_closeness(grid, vox[0]))

    cost = obst_cost + vox[1]
    return cost

def same_point_cond(current, target, grid):
    return np.array_equal(current, target)

def visibility_cond(current, target, grid):
    return observation_range * observation_range > sqr_dist(current, target) and (
        vr.check_intersection(current, target, grid) is None)

def find_path_A_star(grid, start, end,
    stop_cond = same_point_cond):
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
        
        nbrs = nb.get_neighbours(grid, current, D1, D2, D3, H)
        for next in nbrs:
            new_cost = costs[tuple(current)] + get_cost(grid, next)
            next = next[0]
            if (tuple(next) not in costs) or (new_cost < costs[tuple(next)]):
                parents[tuple(next)] = current
                costs[tuple(next)] = new_cost
                next_node = pn.Node(next, new_cost +
                    heuristics(next, end))
                frontier.put((next_node.priority, next_node))
    
    return parents
