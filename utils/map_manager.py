import numpy as np
import open3d as o3d
import os
import utm
import pickle

maps_dir = "maps/"
missions_dir = "missions/"
mission_prefix = "mission_"

pc_file = "map.ply"
pc_info_file = "map.info"
map_occupancy_grid = "occupancy.grid"

def create_project(path, voxel_size):
    filename = os.path.splitext(os.path.basename(path))[0]
    if not os.path.exists(maps_dir): os.mkdir(maps_dir)

    project_dir = maps_dir + filename + '/'
    if not os.path.exists(project_dir):
        os.mkdir(project_dir)
        pcd = o3d.io.read_point_cloud(path)
        pcd_center = pcd.get_center()
        np.savetxt(project_dir + pc_info_file, pcd_center)
        info_file = open(project_dir + pc_info_file, 'r')
        
        info_file.close()
        pcd = pcd.translate((0, 0, 0), relative=False)
        pcd = pcd.voxel_down_sample(voxel_size)
        o3d.io.write_point_cloud(project_dir + pc_file, pcd)
    return project_dir

def get_map_shift(path):
    return np.loadtxt(path + pc_info_file)

def load_occupancy_grid(path):
    file = open(path, 'r')

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
    first_latlon = utm.to_latlon(last[0], last[1], 37, 'N')
    last_latlon = utm.to_latlon(first[0], first[1], 37, 'N')
    file.write(f"1\t0\t3\t22\t0\t0\t0\t0\t{first_latlon[0]}\t{first_latlon[1]}\t{last[2]}\t1\n")
    counter = 2
    for i in range(len(route) - 2, 0, -1):
        pos = route[i]
        latlon = utm.to_latlon(pos[0], pos[1], 37, 'N')
        file.write(f"{counter}\t0\t3\t16\t0\t0\t0\t0\t{latlon[0]}\t{latlon[1]}\t{pos[2]}\t1\n")
        counter += 1
    file.write(f"{counter}\t0\t3\t21\t0\t0\t0\t0\t{last_latlon[0]}\t{last_latlon[1]}\t{first[2]}\t1\n")
    file.close()
