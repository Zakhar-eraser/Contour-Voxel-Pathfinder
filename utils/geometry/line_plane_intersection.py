import numpy as np

def find_intersection(line, plane):
    ld = line[1] - line[0]
    pd2 = plane[1] - plane[0]
    pd3 = plane[2] - plane[0]
    X1 = pd2[1] * pd3[2] - pd3[1] * pd2[2]
    Y1 = pd3[0] * pd2[2] - pd2[0] * pd3[2]
    Z1 = pd2[0] * pd3[1] - pd3[0] * pd2[1]
    a = np.array(((ld[1], -ld[0], 0),
                  (ld[2], 0, -ld[0]),
                  (X1, Y1, Z1)))
    b = np.array((line[0][0] * ld[1] - line[0][1] * ld[0],
                  line[0][0] * ld[2] - line[0][2] * ld[0],
                  X1 * plane[0][0] + Y1 * plane[0][1] + Z1 * plane[0][2]))
    try:
        x = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        x = None
    return x