Object Detection, Association and Mapping for Agiprobot Dataset (C++)

## update 20230318

## Main function
try to detect object in Agiprobot environment

input:
- rgb image
- depth image
- camera information (instrinsic, translation to odom)
- 2d bounding box
- 2d grid maps (when neccessary)

output
- object parameters (translation, orientation and dimension)
- object map

## How to use it 
```c++
mkdir build
cd build
cmake ..
make -j4
./object_seg_node ~/dataset/agiprobot/agiprobot_2
```
we also show final point cloud with "map","result_pcl.ply" and "result_obj.txt", all in odom frame
```c++
./show_hypermap ~/dataset/agiprobot/agiprobot_2
```

## Dataset format: 
the dataset should be like: 
- rgb folder: images with timestamp.png
- depth folder: images with timestamp.png
- labels folder: bbox with timestamp.txt
- maps folder: offline grid map
- Additional.yaml: essential paramter for camera and debug
- associate.txt: timestamp, rgb_img, depth_img, odom
- odom.txt (optinal)
- ps: odom.txt is collected by wheel, which is not accurate, so, we create another odom by laser data. Finally, we use laser odom. 

## 3D Object Detection: 
* read bbox point cloud with detection
    >> before 3D object segmentation, check 2D bbox is good: Vis.showopencv: 1
* segment object point cloud in bbox point cloud: 
    >> for this dataset we use plane segmentation, for other dataset, it does not work
    >> for bbox pcl, we compute plane and collect plane points. (why: conveyor object are actually cuboid)
* save frame objects with 9 DoF and point cloud
    >> to compute the 9 DoF parameters, we transfrom object pcl to world coordinates, where only yaw angle is required.
    >> xyz comes from the centroid, yaw angle are sampled. we project pcl to ground, sample yaw from 0 to 180, and calculate the minimum area that can cover all the projected points, length and width come from the minimum area and height comes from centroid. 
* asssociate frame objects with map objects
    >> for object association, currently it is simple done by label, distance and hsv, maybe add 2D IoU and 3D IoU later.
* update map objects, to merge old map object or initialize new map object
    >> it is tricky to use all history points or just recent 10 frame points
    >> update map objects, need to compute 9 DoF again
* visualize point cloud map with objects
    >> plot bounding box and axises, better visualization
* (optinal) load grid map with point cloud map
    >> just to make sure result match to global map

