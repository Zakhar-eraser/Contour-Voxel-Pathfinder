import numpy as np

def sqr_dist(v1, v2):
    return np.sum(np.square(v1 - v2))

def manh_dist(v1, v2):
    return np.sum(np.abs(v1 - v2))