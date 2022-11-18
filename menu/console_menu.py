import utils.map_manager as mm
import os
import pickle
import structures.map_info as info
from os.path import join
from utils.map_manager import write_waypoints

new_project = "[New project]"
choosing_project = "Choose project to open:"
map_path = "Enter path of a new project`s map ply file:"
voxel_size = "Enter map subsampling voxel size:"
wrong_range = "There are no projects with such index"
wrong_format = "Expected an integer, but entered something else"
accept = "y"
decline = "n"
accept_or_decline = f"[{accept}/{decline}]"
save_waypoints = f"Save found route to .waypoints file?"
wrong_answer_notice = f"'{accept}' or '{decline}' expecting"
ask_for_gps_reference = "Do pointcloud GPS referenced?"

def create_project_dialogue():
    inf = info.Info()
    print(map_path)
    path = input()
    print(voxel_size)
    vs = input()
    if vs.isdigit():
        vs = int(vs)
        inf = mm.create_project(path, vs)
    return inf

def open_or_create_project_dialogue():
    inf = info.Info()
    projects_list = [new_project]
    for project in os.listdir(mm.maps_dir):
        project_dir = join(mm.maps_dir, project)
        if mm.check_project_consistence(project_dir):
            projects_list.append(project)
    
    list_len = len(projects_list)
    print(choosing_project)
    for i in range(list_len):
        print(f"{i}\t{projects_list[i]}")

    selection = input()
    if selection.isdigit():
        selection = int(selection)
        if selection in range(list_len):
            if selection == 0:
                inf = create_project_dialogue()
            else:
                with open(join(join(mm.maps_dir,
                    projects_list[selection]),
                    mm.pc_info_file), 'rb') as info_file:
                    inf = pickle.load(info_file)
        else:
            print(wrong_range)
    else:
        print(wrong_format)
    
    return inf

def acceptance(acc_conseq, dec_conseq = lambda: None):
    cont = True
    print(accept_or_decline)
    while cont:
        ans = input()
        if ans == "y":
            cont = False
            acc_conseq()
        elif ans == "n":
            cont = False
            dec_conseq()
        else:
            print(wrong_answer_notice)

def write_points_dialogue(path, name, route):
    print(save_waypoints, end=' ')
    acceptance(lambda: write_waypoints(path, name, route))

def use_utm_coordinates_dialogue():
    print(ask_for_gps_reference, end=' ')
    def acc():
        mm.gps_ref_use = True
    def dec():
        mm.gps_ref_use = False
    acceptance(acc, dec)
