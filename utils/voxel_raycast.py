import numpy as np

voxel_size = 1

def sqr_dist(v1, v2):
    v = v1 - v2
    return np.sum(np.square(v))

def zStep(pos, cur_voxel, steps, start_pos, bnds, ds):
    cur_voxel[2] += steps[2]
    return list(cur_voxel)

def yStep(pos, cur_voxel, steps, start_pos, bnds, ds):
    cur_voxel[1] += steps[1]
    return list(cur_voxel)

def xStep(pos, cur_voxel, steps, start_pos, bnds, ds):
    cur_voxel[0] += steps[0]
    return list(cur_voxel)

def xyStep(pos, cur_voxel, steps, start_pos, bnds, dds):
    x_by_y = dds[0] * (bnds[1] - start_pos[1]) + start_pos[0]
    y_by_x = dds[1] * (bnds[0] - start_pos[0]) + start_pos[1]
    posXZ = (x_by_y, bnds[1], start_pos[2])
    posYZ = (bnds[0], y_by_x, start_pos[2])
    distXZ = sqr_dist(posXZ, pos)
    distYZ = sqr_dist(posYZ, pos)
    checks = list(cur_voxel)
    if distXZ == distYZ:
        checks.append(cur_voxel + (steps[0], 0, 0))
        checks.append(cur_voxel + (0, steps[1], 0))
        cur_voxel += (steps[0], steps[1], 0)
        pos = posXZ
    elif distXZ < distYZ:
        cur_voxel[1] += steps[1]
        pos = posXZ
    else:
        cur_voxel[0] += steps[0]
        pos = posYZ
    return checks

def yzStep(pos, cur_voxel, steps, start_pos, bnds, dds):
    y_by_z = dds[0] * (bnds[2] - start_pos[2]) + start_pos[1]
    posXY = (start_pos[0], y_by_z, bnds[2])
    distXY = sqr_dist(posXY, pos)
    z_by_y = dds[1] * (bnds[1] - start_pos[1]) + start_pos[2]
    posXZ = (start_pos[0], bnds[1], z_by_y)
    distXZ = sqr_dist(posXZ, pos)
    checks = list(cur_voxel)
    if distXZ == distXY:
        checks.append(cur_voxel + (0, 0, steps[2]))
        checks.append(cur_voxel + (0, steps[1], 0))
        cur_voxel += (0, steps[1], steps[2])
        pos = posXZ
    elif distXZ < distXY:
        cur_voxel[1] += steps[1]
        pos = posXZ
    else:
        cur_voxel[2] += steps[2]
        pos = posXY
    return checks

def xzStep(pos, cur_voxel, steps, start_pos, bnds, dds):
    x_by_z = dds[0] * (bnds[2] - start_pos[2]) + start_pos[0]
    z_by_x = dds[1] * (bnds[0] - start_pos[0]) + start_pos[2]
    posYZ = (bnds[0], start_pos[1], z_by_x)
    posXY = (x_by_z, start_pos[1], bnds[2])
    distYZ = sqr_dist(posYZ, pos)
    distXY = sqr_dist(posXY, pos)
    checks = list(cur_voxel)
    if distYZ == distXY:
        checks.append(cur_voxel + (0, 0, steps[2]))
        checks.append(cur_voxel + (0, steps[1], 0))
        cur_voxel += (steps[0], 0, steps[2])
        pos = posYZ
    elif distYZ < distXY:
        cur_voxel[0] += steps[0]
        pos = posYZ
    else:
        cur_voxel[2] += steps[2]
        pos = posXY
    return checks

def xyzStep(pos, cur_voxel, steps, start_pos, bnds, dds):
    x_by_z = dds[1] * (bnds[2] - start_pos[2]) + start_pos[0]
    y_by_z = dds[3] * (bnds[2] - start_pos[2]) + start_pos[1]
    posXY = (x_by_z, y_by_z, bnds[2])

    y_by_x = dds[2] * (bnds[0] - start_pos[0]) + start_pos[1]
    z_by_x = dds[4] * (bnds[0] - start_pos[0]) + start_pos[2]
    posYZ = (bnds[0], y_by_x, z_by_x)

    x_by_y = dds[0] * (bnds[1] - start_pos[1]) + start_pos[0]
    z_by_y = dds[5] * (bnds[1] - start_pos[1]) + start_pos[2]
    posXZ = (x_by_y, bnds[1], z_by_y)
    
    distXY = sqr_dist(posXY, pos)
    distYZ = sqr_dist(posYZ, pos)
    distXZ = sqr_dist(posXZ, pos)
    checks = list(cur_voxel)
    if (distXY == distYZ) and (distXY == distXZ):
        checks.append(cur_voxel + (steps[0], 0, 0))
        checks.append(cur_voxel + (0, steps[1], 0))
        checks.append(cur_voxel + (0, 0, steps[2]))
        cur_voxel += steps
        pos = posXY
    elif distXY == distYZ:
        if distXY < distXZ:
            checks.append(cur_voxel + (steps[0], 0, 0))
            checks.append(cur_voxel + (0, 0, steps[2]))
            cur_voxel += (steps[0], 0, steps[2])
            pos = posXY
        else:
            cur_voxel[1] += steps[1]
            pos = posXZ
    elif distYZ == distXZ:
        if distYZ < distXY:
            checks.append(cur_voxel + (steps[0], 0, 0))
            checks.append(cur_voxel + (0, steps[1], 0))
            cur_voxel += (steps[0], steps[1], 0)
            pos = posYZ
        else:
            cur_voxel[2] += steps[2]
            pos = posXY
    elif distXZ == distXY:
        if distXZ < distYZ:
            checks.append(cur_voxel + (0, steps[1], 0))
            checks.append(cur_voxel + (0, 0, steps[2]))
            cur_voxel += (0, steps[1], steps[2])
            pos = posXZ
        else:
            cur_voxel[0] += steps[0]
            pos = posYZ
    elif (distXY < distYZ) and (distXY < distXZ):
        cur_voxel[2] += steps[2]
        pos = posXY
    elif (distYZ < distXY) and (distYZ < distXZ):
        cur_voxel[0] += steps[0]
        pos = posYZ
    else:
        cur_voxel[1] += steps[1]
        pos = posXZ
    return checks

def check_intersection(start_voxel, target_voxel, occupancy_grid):
    inter_vox = None
    cur_voxel = np.copy(start_voxel)
    pos = np.copy(start_voxel)
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
        dds = (dX/dY, dX/dZ, dY/dX, dY/dZ, dZ/dX, dZ/dY)
    
    while (inter_vox is None) and (not np.array_equal(cur_voxel, target_voxel)):
        check_voxels = step_fun(pos, cur_voxel, steps, start_voxel, cur_voxel + B, dds)
        for vox in check_voxels:
            if occupancy_grid[tuple(vox)]:
                inter_vox = vox
    return inter_vox