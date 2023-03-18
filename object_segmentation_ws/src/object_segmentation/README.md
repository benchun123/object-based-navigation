Object Mapping for Agiprobot Dataset (ROS)

## update 20230316

## Main function
try to detect object in Agiprobot ROS Bag, mainly to generate hypermap(grid map with object information)

input:
- rosbag (topic: rgb, depth, camera_info, tf)
- bbox folder (bbox detection with timestamp.txt)
- 2d grid maps 
- config.yaml(calibrated transfrom and debug parameters)

output: 
- map in rviz
- publish topics: raw_pcl, obj_pcl, map, tf

## How to use it
Prerequisites
    Ubuntu (18.04.5)
    ROS melodic
    OpenCV (3.2.0)
    PCL (1.8.1)

in one terminal: 
```c++
cd object_segmentation_ws
catkin_make
source devel/setup.bash
roslaunch object_segmentation debug.launch
```
in another terminal where we have the bag
```c++
rosbag play agiprobot_3.bag
```
## save object information
```c++
rostopic pub /save_map_cuboid std_msgs/Bool "data: false" 
```
the global object information(in map frame) will save to 
$(find object_segmentation)/config/result_obj.txt

you can also show the object information with map by 
```c++
roslaunch object_segmentation show_hyper_map.launch 
```

## 3D Object Detection: 
* read bbox point cloud with detection
    >> every image callback, we search for the detection with same timestamp, and read offline detection
* segment object point cloud in bbox point cloud: 
    >> for this dataset we use plane segmentation, for other dataset, it does not work
    >> for bbox pcl, we compute plane and collect plane points. (why: conveyor object are actually cuboid)
* save frame objects with 9 DoF, publish label point cloud
    >> to compute the 9 DoF parameters, we transfrom object pcl to world coordinates, where only yaw angle is required.
    >> xyz comes from the centroid, yaw angle are sampled. we project pcl to ground, sample yaw from 0 to 180, and calculate the minimum area that can cover all the projected points, length and width come from the minimum area and height comes from centroid. 
    >> publish object point cloud with label. topic: object_segmentation, which can be modified in .launch file
* asssociate frame objects with map objects
    >> for object association, currently it is simple done by label, distance and hsv, maybe add 2D IoU and 3D IoU later.
* update map objects, to merge old map object or initialize new map object
    >> it is tricky to use all history points or just recent 10 frame points
    >> update map objects, need to compute 9 DoF again
* visualize grid map with objects
    >> load 2d grid map by map server in launch file.
    >> publish global object pcl in map (switch frame or global in config.yaml)
    >> publish project object plane in map (hypermap, global item) 
    

## Comparison between C++ Code and ROS Code
* main code are same: object detection, association, mapping components
* difference 1: input:
  - C++: all in file
  - ROS: all in bag (the detection should be also included in rosbag)
* differece 2: visualization
  - C++: pcl visualzer, should current info is difficult, but easy to debug
  - ROS: rviz, can see both current and global info, add more plug in.
* differece 3: save object information 
  - C++: save object.txt, object.ply, raw.ply to dataset folder
  - ROS: save object.txt to config folder
