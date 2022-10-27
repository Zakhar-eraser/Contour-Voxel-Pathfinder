import numpy as np
import laspy as lp
import open3d as o3d
import os
import utm

maps_dir = "maps/"
missions_dir = "missions/"
mission_prefix = "mission_"

pc_name = "map.ply"

def create_project(path, voxel_size):
    filename = os.path.splitext(os.path.basename(path))[0]
    if not os.path.exists(maps_dir): os.mkdir(maps_dir)

    project_dir = maps_dir + filename + '/'
    if not os.path.exists(project_dir):
        os.mkdir(project_dir)
        lp_pcd = lp.read(path)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.vstack((lp_pcd.X, lp_pcd.Y, lp_pcd.Z)).transpose())
        pcd.colors = o3d.utility.Vector3dVector(np.vstack((lp_pcd.red, lp_pcd.green, lp_pcd.blue)).transpose()/65535)
        #pcd = o3d.io.read_point_cloud(path)
        pcd = pcd.voxel_down_sample(voxel_size)
        o3d.io.write_point_cloud(project_dir + pc_name, pcd)
    return project_dir

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
    file.write(f"1\t0\t3\t21\t0\t0\t0\t0\t{last_latlon[0]}\t{last_latlon[1]}\t{first[2]}\t1\n")
    file.close()
