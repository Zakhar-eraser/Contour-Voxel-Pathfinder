import numpy as np

voxel_size = 1

def sqr_dist(v1, v2):
    v = v1 - v2
    return np.sum(np.square(v))

def check_intersection1(start_voxel, target_voxel, occupancy_grid):
    tmp = occupancy_grid[tuple(target_voxel)]
    occupancy_grid[tuple(target_voxel)] = 0
    intersect = False
    cur_voxel = start_voxel
    x1 = start_voxel[0]
    y1 = start_voxel[1]
    z1 = start_voxel[2]
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

    while intersect == False and not np.array_equal(cur_voxel, target_voxel):
        bnds = cur_voxel + B
        if dX == 0 and dY == 0:
            cur_voxel[2] += sZ
        elif dX == 0 and dZ == 0:
            cur_voxel[1] += sY
        elif dY == 0 and dZ == 0:
            cur_voxel[0] += sX
        elif dX == 0:
            y_by_z = dY/dZ * (bnds[2] - z1)
            distXY = sqr_dist((x1, y_by_z, bnds[2]), cur_voxel)
            z_by_y = dZ/dY * (bnds[1] - y1)
            distYZ = sqr_dist((x1, bnds[1], z_by_y), cur_voxel)
            if distYZ == distXY:
                cur_voxel += (0, sY, sZ)
            elif distYZ < distXY:
                cur_voxel[1] += sY
            else:
                cur_voxel[2] += sZ
        elif dY == 0:
            x_by_z = dY/dZ * (bnds[2] - z1)
            z_by_x = dZ/dX * (bnds[0] - x1)
            distXZ = sqr_dist((bnds[0], y1, z_by_x), cur_voxel)
            distXY = sqr_dist((x_by_z, y1, bnds[2]))
            if distXZ == distXY:
                cur_voxel += (sX, 0, sZ)
            elif distXZ < distXY:
                cur_voxel[0] += sX
            else:
                cur_voxel[2] += sZ
        elif dZ == 0:
            
            
        #x_by_y = dXdY * (bnds[1] - y1)
        #x_by_z = dXdZ * (bnds[2] - z1)
        #y_by_x = dYdX * (bnds[0] - x1)
        #y_by_z = dYdZ * (bnds[2] - z1)
        #z_by_x = dZdX * (bnds[0] - x1)
        #z_by_y = dZdY * (bnds[1] - y1)

        distXY = sqr_dist([x_by_z, y_by_z, bnds[2]], cur_voxel)
        distYZ = sqr_dist([bnds[0], y_by_x, z_by_x], cur_voxel)
        distXZ = sqr_dist([x_by_y, bnds[1], z_by_y], cur_voxel)

        if (distXY <= distXZ) and (distXY <= distYZ):
            cur_voxel[2] += sZ
        if (distXZ <= distXY) and (distXZ <= distYZ):
            cur_voxel[1] += sY
        if (distYZ <= distXZ) and (distYZ <= distXY):
            cur_voxel[0] += sX
        
        if occupancy_grid[tuple(cur_voxel)]:
            intersect = True
    occupancy_grid[tuple(target_voxel)] = tmp
    return intersect