import numpy as np

class VoxelNode:
    """An object of the class contains voxel`s indexes and cost of movement to the voxel"""
    def ___init___(self, idx, parent, cost):
        self.idx = idx
        self.parent = parent
        self.cost = cost
    
    def __eq__(self, __o: object) -> bool:
        return self.idx == __o.idx
    
    def get_neigbours(self):
        fwd = self.idx + np.array([1, 0, 0], dtype=np.int32)
        bwd = self.idx + np.array([-1, 0, 0], dtype=np.int32)
        rgt = self.idx + np.array([0, 1, 0], dtype=np.int32)
        lft = self.idx + np.array([0, -1, 0], dtype=np.int32)
        up = self.idx + np.array([0, 0, 1], dtype=np.int32)
        dwn = self.idx + np.array([0, 0, -1], dtype=np.int32)
        fwd_rgt_up = self.idx + np.array([1, 1, 1], dtype=np.int32)
        bwd_rgt_up = self.idx + np.array([-1, 1, 1], dtype=np.int32)
        fwd_lft_up = self.idx + np.array([1, -1, 1], dtype=np.int32)
        bwd_lft_up = self.idx + np.array([-1, -1, 1], dtype=np.int32)
        fwd_rgt_dwn = self.idx + np.array([1, 1, -1], dtype=np.int32)
        bwd_rgt_dwn = self.idx + np.array([-1, 1, -1], dtype=np.int32)
        fwd_lft_dwn = self.idx + np.array([1, -1, -1], dtype=np.int32)
        bwd_lft_dwn = self.idx + np.array([-1, -1, -1], dtype=np.int32)
        return [fwd, bwd, rgt, lft, up, dwn, fwd_rgt_up, fwd_rgt_dwn,
            bwd_rgt_dwn, bwd_rgt_up, fwd_lft_dwn, fwd_lft_up, bwd_lft_dwn, bwd_lft_up]
    

def find_path_A_star(grid, start, end, stop_condition):
