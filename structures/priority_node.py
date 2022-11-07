class Node:
    """Interface to work numpy array with PriorityQueue"""

    def __init__(self, idx, priority):
        self.idx = idx
        self.priority = priority
    
    def __repr__(self) -> str:
        return str((self.priority, self.idx))

    def __eq__(self, other):
        return self.priority == other.priority
    
    def __hash__(self):
        return hash(tuple(self.idx))

    def __lt__(self, other):
        return self.priority < other.priority