class Info:
    """Contain map`s shift, hash and voxel size"""

    def __init__(self, hash = None, shift = None,
                 voxel_size = None, project_dir = None):
        self.hash = hash
        self.shift = shift
        self.voxel_size = voxel_size
        self.project_dir = project_dir