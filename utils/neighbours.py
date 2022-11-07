import numpy as np

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