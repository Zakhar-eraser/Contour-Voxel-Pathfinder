import numpy as np

voxel_size = 1

def sqr_dist(v1, v2):
    v = v1 - v2
    return np.sum(np.square(v))

def zStep(pos, steps, start_pos, bnds, ds):
    pos[2] += steps[2]

def yStep(pos, steps, start_pos, bnds, ds):
    pos[1] += steps[1]

def xStep(pos, steps, start_pos, bnds, ds):
    pos[0] += steps[0]

def xyStep(pos, steps, start_pos, bnds, dds):
    x_by_y = dds[0] * (bnds[1] - start_pos[1])
    y_by_x = dds[1] * (bnds[0] - start_pos[0])
    distXZ = sqr_dist((bnds[0], y_by_x, start_pos[2]), pos)
    distYZ = sqr_dist((x_by_y, bnds[1], start_pos[2]), pos)
    if distXZ == distYZ:
        pos += (steps[0], steps[1], 0)
    elif distXZ < distYZ:
        pos[0] += steps[0]
    else:
        pos[1] += steps[1]

def yzStep(pos, steps, start_pos, bnds, dds):
    y_by_z = dds[0] * (bnds[2] - start_pos[2])
    distXY = sqr_dist((start_pos[0], y_by_z, bnds[2]), pos)
    z_by_y = dds[1] * (bnds[1] - start_pos[1])
    distYZ = sqr_dist((start_pos[0], bnds[1], z_by_y), pos)

    if distYZ == distXY:
        pos += (0, steps[1], steps[2])
    elif distYZ < distXY:
        pos[1] += steps[1]
    else:
        pos[2] += steps[2]

def xzStep(pos, steps, start_pos, bnds, dds):
    x_by_z = dds[0] * (bnds[2] - start_pos[2])
    z_by_x = dds[1] * (bnds[0] - start_pos[0])
    distXZ = sqr_dist((bnds[0], start_pos[1], z_by_x), pos)
    distXY = sqr_dist((x_by_z, start_pos[1], bnds[2]), pos)

    if distXZ == distXY:
        pos += (steps[0], 0, steps[2])
    elif distXZ < distXY:
        pos[0] += steps[0]
    else:
        pos[2] += steps[2]

def xyzStep(pos, steps, start_pos, bnds, dds):
    x_by_z = dds[0] * (bnds[2] - start_pos[2])
    y_by_z = dds[1] * (bnds[2] - start_pos[2])
    z_by_x = dds[2] * (bnds[0] - start_pos[0])
    distXY = sqr_dist((x_by_z, y_by_z, bnds[2]), pos)
    distYZ = sqr_dist((bnds[0], y_by_z, z_by_x), pos)
    distXZ = sqr_dist((x_by_z, bnds[1], z_by_x), pos)

    if (distXY == distYZ) and (distXY == distXZ):
        pos += steps
    elif distXY == distYZ:
        if distXY < distXZ:
            pos += (0, steps[1], steps[2])
        else: pos[0] += steps[0]
    elif distYZ == distXZ:
        if distYZ < distXY:
            pos += (steps[0], steps[1], 0)
        else: pos[2] += steps[2]
    elif distXZ == distXY:
        if distXZ < distYZ:
            pos += (steps[0], 0, steps[2])
        else: pos[1] += steps[1]
    elif (distXY < distYZ) and (distXY < distXZ):
        pos[2] += steps[2]
    elif (distYZ < distXY) and (distYZ < distXZ):
        pos[1] += steps[1]
    else:
        pos[0] += steps[0]

def check_intersection(start_voxel, target_voxel, occupancy_grid):
    tmp = occupancy_grid[tuple(target_voxel)]
    occupancy_grid[tuple(target_voxel)] = 0
    intersect = False
    cur_voxel = np.copy(start_voxel)
    dX = target_voxel[0] - start_voxel[0]
    dY = target_voxel[1] - start_voxel[1]
    dZ = target_voxel[2] - start_voxel[2]
    vox_step = voxel_size / 2
    B = np.full(3, fill_value=vox_step, dtype=np.float32)
    if dX < 0: B[0] = -B[0]
    if dY < 0: B[1] = -B[1]
    if dZ < 0: B[2] = -B[2]
    sX = 1 if dX > 0 else -1
    sY = 1 if dY > 0 else -1
    sZ = 1 if dZ > 0 else -1
    steps = (sX, sY, sZ)
    if dX == 0 and dY == 0:
        step_fun = zStep
        dds = None
    elif dX == 0 and dZ == 0:
        step_fun = yStep
        dds = None
    elif dY == 0 and dZ == 0:
        step_fun = xStep
        dds = None
    elif dX == 0:
        step_fun = yzStep
        dds = (dY/dZ, dZ/dY)
    elif dY == 0:
        step_fun = xzStep
        dds = (dX/dZ, dZ/dX)
    elif dZ == 0:
        step_fun = xyStep
        dds = (dX/dY, dY/dX)
    else:
        step_fun = xyzStep
        dds = (dX/dZ, dY/dZ, dZ/dX)
        
    while (intersect == False) and (not np.array_equal(cur_voxel, target_voxel)):
        step_fun(cur_voxel, steps, start_voxel, cur_voxel + B, dds)
        if occupancy_grid[tuple(cur_voxel)]:
            intersect = True
    occupancy_grid[tuple(target_voxel)] = tmp
    return intersect