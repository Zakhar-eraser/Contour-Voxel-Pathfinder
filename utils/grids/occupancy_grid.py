import numpy as np

def pos2idx(min_bound, pos, voxel_size):
    return ((pos - min_bound) /
        voxel_size).astype('int32') + 1

def idx2pos(min_bound, idx, voxel_size):
    return (idx - 1) * voxel_size + min_bound + voxel_size / 2

def get_occupancy_grid(voxel_grid):
    bb = voxel_grid.get_axis_aligned_bounding_box()
    shape = (bb.get_extent() / voxel_grid.voxel_size).astype('int32')
    grid = np.zeros(tuple(shape + 2), dtype=np.int8)
    grid[:, :, 0] = 1
    grid[:, :, shape[2] + 1] = 1
    grid[0, :, 1:shape[2] + 1] = 1
    grid[shape[0] + 1, :, 1:shape[2] + 1] = 1
    grid[1:shape[0] + 1, 0, 1:shape[2] + 1] = 1
    grid[1:shape[0] + 1, shape[1] + 1, 1:shape[2] + 1] = 1
    
    for vox in voxel_grid.get_voxels():
        grid[tuple(vox.grid_index + 1)] = 1
    return grid
