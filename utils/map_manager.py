import numpy as np
import open3d as o3d
import os
from os.path import exists
from os.path import join
import utm
import pickle
import structures.map_info as info
import hashlib
import utils.grids.occupancy_grid as ocg

maps_dir = "maps/"
missions_dir = "missions/"
mission_prefix = "mission_"

pc_file = "map.ply"
pc_info_file = "map.info"
map_occupancy_grid = "occupancy_grid.npy"

gps_ref_use = True

def init_project_structure():
    if not exists(maps_dir): os.mkdir(maps_dir)

def hash(path):
    md5_hash = hashlib.md5()
    with open(path,"rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            md5_hash.update(byte_block)
    return md5_hash.hexdigest()

def create_project(path, voxel_size):
    inf = info.Info()
    if exists(path):
        filename = os.path.splitext(os.path.basename(path))[0]
        project_dir = maps_dir + filename + '/'
        file_path = project_dir + pc_info_file

        if exists(file_path):
            with open(file_path, "rb") as info_file:
                inf = pickle.load(info_file)
        
        print("Calculating hash...")

        hs = hash(path)
        if (inf.hash != hs) or (inf.voxel_size != voxel_size) or (
            not exists(join(project_dir, map_occupancy_grid))):
            if not exists(project_dir):
                os.mkdir(project_dir)
            pcd = o3d.io.read_point_cloud(path)
            pcd_center = pcd.get_center()

            inf = info.Info(
                hs,
                pcd_center,
                voxel_size,
                project_dir)
            with open(file_path, "wb") as info_file:
                pickle.dump(inf, info_file)

            print("Translating point cloud to zero...")

            pcd = pcd.translate((0, 0, 0), relative=False)

            print("Point cloud downsampling...")

            pcd = pcd.voxel_down_sample(voxel_size)

            print("Creating voxel grid...")

            grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

            print("Saving occupancy grid")
            with open(join(project_dir, map_occupancy_grid), 'wb') as file:
                np.save(file, ocg.get_occupancy_grid(grid))

            print("Saving subsampled point cloud...")

            o3d.io.write_point_cloud(project_dir + pc_file, pcd)
    else:
        print("It can`t be opened a map ply file on path:")
        print(path)
    return inf

def check_project_consistence(path):
    consistant = False
    if os.path.isdir(path):
        info = join(path, pc_info_file)
        ply = join(path, pc_file)
        grid = join(path, map_occupancy_grid)
        if exists(info) and exists(ply) and exists(grid):
            consistant = True
    return consistant

def write_waypoints(path, name, route):
    ms_dir = path + missions_dir
    if not os.path.exists(ms_dir):
        os.mkdir(ms_dir)
    m_count = len(os.listdir(ms_dir))
    m_dir = ms_dir + mission_prefix + str(m_count + 1) + "/"
    os.mkdir(m_dir)
    file = open(m_dir + name + ".waypoints", "w")
    file.write("QGC WPL 110\n")
    file.write("0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1\n")
    last = route[len(route) - 1]
    first = route[0]
    if gps_ref_use:
        first_latlon = utm.to_latlon(last[0], last[1], 37, 'N')
        last_latlon = utm.to_latlon(first[0], first[1], 37, 'N')
    else:
        first_latlon = (last[0], last[1])
        last_latlon = (first[0], first[1])
    file.write(f"1\t0\t3\t22\t0\t0\t0\t0\t{first_latlon[0]}\t{first_latlon[1]}\t{last[2]}\t1\n")
    counter = 2
    for i in range(len(route) - 2, 0, -1):
        pos = route[i]
        if gps_ref_use:
            latlon = utm.to_latlon(pos[0], pos[1], 37, 'N')
        else:
            latlon =(pos[0], pos[1])
        file.write(f"{counter}\t0\t3\t16\t0\t0\t0\t0\t{latlon[0]}\t{latlon[1]}\t{pos[2]}\t1\n")
        counter += 1
    file.write(f"{counter}\t0\t3\t21\t0\t0\t0\t0\t{last_latlon[0]}\t{last_latlon[1]}\t{first[2]}\t1\n")
    file.close()
