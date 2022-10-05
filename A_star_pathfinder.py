import numpy as np
from queue import PriorityQueue

class VoxelNode:
    """An object of the class contains voxel`s indexes and cost of movement to the voxel"""

    move_cost = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 3.0]])

    def ___init___(self, idx, parent, cost):
        self.idx = idx
        self.parent = parent
        self.cost = cost
    
    def __eq__(self, __o: object) -> bool:
        return self.idx == __o.idx
    
    def __hash__(self) -> int:
        return hash((self.idx[0], self.idx[1], self.idx[2]))

    def get_neigbours(self):
        mc = VoxelNode.move_cost
        fwd = VoxelNode(self.idx + np.array([1, 0, 0]), self.idx, mc[0, 0])
        bwd = VoxelNode(self.idx + np.array([-1, 0, 0]), self.idx, mc[1, 0])
        rgt = VoxelNode(self.idx + np.array([0, 1, 0]), self.idx, mc[0, 1])
        lft = VoxelNode(self.idx + np.array([0, -1, 0]), self.idx, mc[1, 1])
        up = VoxelNode(self.idx + np.array([0, 0, 1]), self.idx, mc[0, 2])
        dwn = VoxelNode(self.idx + np.array([0, 0, -1]), self.idx, mc[1, 2])
        fwd_up = VoxelNode(self.idx + np.array([1, 0, 1]), self.idx, mc[0, 0] + mc[0, 2])
        fwd_dwn = VoxelNode(self.idx + np.array([1, 0, -1]), self.idx, mc[0, 0] + mc[1, 2])
        bwd_up = VoxelNode(self.idx + np.array([-1, 0, 1]), self.idx, mc[1, 0] + mc[0, 2])
        bwd_dwn = VoxelNode(self.idx + np.array([-1, 0, -1]), self.idx, mc[1, 0] + mc[1, 2])
        fwd_rgt = VoxelNode(self.idx + np.array([1, 1, 0]), self.idx, mc[0, 0] + mc[0, 1])
        fwd_lft = VoxelNode(self.idx + np.array([1, -1, 0]), self.idx, mc[0, 0] + mc[1, 1])
        bwd_rgt = VoxelNode(self.idx + np.array([-1, 1, 0]), self.idx, mc[1, 0] + mc[0, 1])
        bwd_lft = VoxelNode(self.idx + np.array([-1, -1, 0]), self.idx, mc[1, 0] + mc[1, 1])
        rgt_up = VoxelNode(self.idx + np.array([0, 1, 1]), self.idx, mc[0, 1] + mc[0, 2])
        rgt_dwn = VoxelNode(self.idx + np.array([0, 1, -1]), self.idx, mc[0, 1] + mc[1, 2])
        lft_up = VoxelNode(self.idx + np.array([0, -1, 1]), self.idx, mc[1, 1] + mc[0, 2])
        lft_dwn = VoxelNode(self.idx + np.array([0, -1, -1]), self.idx, mc[1, 1] + mc[1, 2])
        fwd_rgt_up = VoxelNode(self.idx + np.array([1, 1, 1]), self.idx, mc[0, 2] + mc[0, 1] + mc[0, 0])
        bwd_rgt_up = VoxelNode(self.idx + np.array([-1, 1, 1]), self.idx, mc[1, 2] + mc[0, 1] + mc[0, 0])
        fwd_lft_up = VoxelNode(self.idx + np.array([1, -1, 1]), self.idx, mc[0, 0] + mc[1, 1] + mc[0, 2])
        bwd_lft_up = VoxelNode(self.idx + np.array([-1, -1, 1]), self.idx, mc[1, 0] + mc[1, 1] + mc[0, 2])
        fwd_rgt_dwn = VoxelNode(self.idx + np.array([1, 1, -1]), self.idx, mc[0, 0] + mc[0, 1] + mc[1, 2])
        bwd_rgt_dwn = VoxelNode(self.idx + np.array([-1, 1, -1]), self.idx, mc[1, 0] + mc[0, 1] + mc[1, 2])
        fwd_lft_dwn = VoxelNode(self.idx + np.array([1, -1, -1]), self.idx, mc[0, 0] + mc[1, 1] + mc[1, 2])
        bwd_lft_dwn = VoxelNode(self.idx + np.array([-1, -1, -1]), self.idx, mc[1, 0] + mc[1, 1] + mc[1, 2])
        return [fwd, bwd, rgt, lft, up, dwn, fwd_rgt_up, fwd_rgt_dwn,
            fwd_up, fwd_dwn, bwd_up, bwd_dwn, fwd_rgt, fwd_lft, bwd_rgt, bwd_lft,
            rgt_up, rgt_dwn, lft_up, lft_dwn,
            bwd_rgt_dwn, bwd_rgt_up, fwd_lft_dwn, fwd_lft_up, bwd_lft_dwn, bwd_lft_up]
    

def find_path_A_star(grid, start, end, stop_condition):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
       current = frontier.get()

       if current == goal:
          break
      
       for next in graph.neighbors(current):
          new_cost = cost_so_far[current] + graph.cost(current, next)
          if next not in cost_so_far or new_cost < cost_so_far[next]:
             cost_so_far[next] = new_cost
             priority = new_cost + heuristic(goal, next)
             frontier.put(next, priority)
             came_from[next] = current