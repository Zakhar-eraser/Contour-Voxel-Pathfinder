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

def get_plane_cardinals(lgrid, idx, cost):
    nbrs = []
    for i in range(2):
        for j in (-1, 1):
            lidx = np.roll((j, 0, 0), shift=i)
            if not lgrid[tuple(lidx + 1)]:
                nbrs.append((lidx + idx, cost))
    return nbrs

def get_height_cardinals(lgrid, idx, cost):
    nbrs = []
    if not lgrid[1, 1, 2]: nbrs.append((idx + (0, 0, 1), cost))
    if not lgrid[1, 1, 0]: nbrs.append((idx + (0, 0, -1), cost))
    return nbrs

def get_plane_diagonals(lgrid, idx, cost):
    nbrs = []
    for i in (1, -1):
        for j in (1, -1):
            lidx = np.array((i, j, 0), dtype=np.int8)
            if not lgrid[tuple(lidx + 1)]:
                nbrs.append((idx + lidx, cost))
    return nbrs

def get_height_diagonals(lgrid, idx, cost):
    nbrs = []
    for i in range(-1, 1):
        for j in range(-1, 1):
            for s in range(2):
                lidx = np.roll((0, i, j), shift=s)
                if not lgrid[tuple(lidx + 1)]:
                    nbrs.append((idx + lidx, cost))
    return nbrs

def get_corners(lgrid, idx, cost):
    nbrs = []
    for i in (1, -1):
        for j in (1, -1):
            for k in (1, -1):
                lidx = np.array((i, j, k), dtype=np.int8)
                if not lgrid[tuple(lidx + 1)]:
                    nbrs.append((idx + lidx, cost))
    return nbrs

def get_neighbours(grid, idx, D1, D2, D3, H):
    local_grid = np.copy(
        grid[idx[0] - 1:idx[0] + 2, idx[1] - 1:idx[1] + 2, idx[2] - 1:idx[2] + 2])
    nbrs_grid = get_neighbours_occupancy(local_grid)
    nbrs = get_plane_cardinals(local_grid, idx, D1) +\
           get_plane_diagonals(local_grid, idx, D2) +\
           get_height_cardinals(local_grid, idx, D1 * H) +\
           get_height_diagonals(local_grid, idx, D2 * H) +\
           get_corners(local_grid, idx, D3 * H)
    return nbrs