from enum import Enum
from numpy import array_equal
from utils.grids.occupancy_grid import qpos2idx

class Transfer(Enum):
    """Enumerator of transfer types"""
    OBSERVE = 0
    VISIT = 1
    START = 2
    TAKEOFF = 3

def exists_idx(target, idx):
    exists = False
    while (target is not None) and (not exists):
        exists = array_equal(target.idx, idx)
        if not exists: target = target.target
    return target

class Targets:
    """Linked List with position and transfer type properties"""

    def __init__(self, mark, name, id):
        self.mark = mark
        self.name = name
        self.id = id
        self.idx = qpos2idx(mark.get_center())
        self.transfer = None
        self.target = None
        self.origin = None
    
    def add(self, target, transfer):
        self.target = target
        self.target.transfer = transfer
        self.target.origin = self
        return self.target

    def move(self, shift):
        self.mark.translate(shift)
        self.idx = qpos2idx(self.mark.get_center())
