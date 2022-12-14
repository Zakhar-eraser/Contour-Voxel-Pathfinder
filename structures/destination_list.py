from enum import Enum

class Transfer(Enum):
    """Enumerator of transfer types"""
    OBSERVE = 0
    DESTINATE = 1

class Path:
    """Linked List with position and transfer type properties"""

    def __init__(self, mark, name):
        self.mark = mark
        self.name = name
        self.transfer = None
        self.target = None
        self.origin = None
    
    def add(self, target, transfer):
        self.target = target
        self.target.transfer = transfer
        self.target.origin = self
        return self.target
