# Contour Voxel Pathfinder
![Open3D 21 11 2022 14_09_18](https://user-images.githubusercontent.com/57917936/203037460-e4f09149-f22c-4569-b3e4-7166a54353bd.png)

## Brief
This app is the Contour project pathfinder that based on A* algorithm with Open3d visualization. Shortest path search performs on a voxel grid.
## Input
.ply point cloud file
## Output
Mavlink .waypoints commands
## Load or create project
Loading and creating app`s projects realized via console dialogues. Dialogues sequence:
1. Select an existing project or create new
  * If an existing project was selected, Open3d visualize the map immediately
2. Insert path of .ply file
3. Enter size of voxel for downsampling
## Selecting points of interest
![Select points 21 11 2022 14_13_47](https://user-images.githubusercontent.com/57917936/203042569-cfba5211-b270-4fc8-b5c4-799cee685125.png)

1. Select start point, which realized by green sphere in visualization, with CTRL + LEFT BUTTON MOUSE click on a desired voxel. If box with "stop when observable" title is checked end point of pathfinder algorithm will be changed to 'target observable from current position'.
If such mode turned on pathfinder will prefer moves in horizontal plane. The main idea of that function is to find nearest to such position, from which it able to observe target position, on a safe altitude.
2. Move sphere in horizontal plane by keyboard ARROWS. Change sphere altitude by keyboard buttons W and S.
3. When it`s done close the window of visualizer.
## Found route visualization
![Open3D 21 11 2022 14_14_55](https://user-images.githubusercontent.com/57917936/203045995-fcfb8b97-a865-4d98-b28f-8adae83b8562.png)

1. Close the window of visualizer when you stop to admire of a calculated route.
## Save route
Console again...
1. Did the .ply have reference to GPS? If not, route will be represented relative (from center of cloud) frame.
App can convert (x, y) to (latitude, longitude) correctly on 37 N UTM zone at the time. It easily may be specified in the next release.
2. Save or not the route to MAVlink commands (it`s the only thing you can save at the time)
.waypoints file will be placed in __maps/<MAP_NAME>/missions/<MISSION_NAME>/__
