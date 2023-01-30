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

class WritableTargets:
    """Targets class for pickle"""

    def __init__(self):
        self.pos = None
        self.name = None
        self.id = None
        self.idx = None
        self.transfer = None
        self.target = None
        self.origin = None

def get_marks(targets):
    marks = []
    while targets is not None:
        marks.append(targets.mark)
        targets = targets.target
    return marks

def get_writeble_targets(targets):
    root = WritableTargets()
    wtarget = root
    while targets is not None:
        wtarget.pos = targets.mark.get_center()
        wtarget.idx = targets.idx
        wtarget.name = targets.name
        wtarget.id = targets.id
        wtarget.transfer = targets.transfer
        if targets.target is not None:
            wtarget.target = WritableTargets()
            wtarget.target.origin = wtarget
        wtarget = wtarget.target
        targets = targets.target
    return root

def end(target):
    if target is not None:
        while target.target is not None:
            target = target.target
    return target