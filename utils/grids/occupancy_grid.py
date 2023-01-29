import numpy as np

mb = np.array([0, 0, 0])
vs = 1

def set_static_voxel_size(voxel_size):
    global vs
    vs = voxel_size

def set_static_min_bound(min_bound):
    global mb
    mb = min_bound

def pos2idx(min_bound, pos, voxel_size):
    return ((pos - min_bound) /
        voxel_size).astype('int32') + 1

def qpos2idx(pos):
    return pos2idx(mb, pos, vs)

def idx2pos(min_bound, idx, voxel_size):
    return (idx - 1) * voxel_size + min_bound + voxel_size / 2

def qidx2pos(idx):
    return idx2pos(mb, idx, vs)

def vect_pos2idx(min_bound, pos_vect, voxel_size):
    if len(pos_vect):
        vfunc = np.vectorize(lambda x: pos2idx(min_bound, x, voxel_size), signature='(n)->(n)')
        pos_vect = vfunc(pos_vect)
    return pos_vect

def qvect_pos2idx(pos_vect):
    return vect_pos2idx(mb, pos_vect, vs)

def vect_idx2pos(min_bound, idx_vect, voxel_size):
    if len(idx_vect):
        vfunc = np.vectorize(lambda x: idx2pos(min_bound, x, voxel_size), signature='(n)->(n)')
        idx_vect = vfunc(idx_vect)
    return idx_vect

def qvect_idx2pos(idx_vect):
    return vect_idx2pos(mb, idx_vect, vs)

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
