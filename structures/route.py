class Route:
    """List with array of points to destinate next point as value"""
    
    def __init__(self):
        self.next_point = None
        self.point = None
        self.visit_points = []
        self.observe_points = []