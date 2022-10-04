class VoxelNode:
    """An object of the class contains voxel`s indexes and cost of movement to the voxel"""
    def ___init___(self, idx, cost):
        self.idx = idx
        self.cost = cost
    

def find_path_A_star(grid, start, end, stop_condition):