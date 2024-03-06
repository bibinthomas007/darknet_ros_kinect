# darknet_ros
## Overview
A package that combines darknet with Kinect to obtain the 3D coordinates of objects.

## Install

```
$ git clone --recursive https://github.com/rionehome/darknet_ros.git
$ cd ../../
$ catkin_make
```
`freenect` and the PointCloud Library (PCL) are required to obtain 3D information.

To install PCL, you can use the following command:

```
$ sudo apt-get update
$ sudo apt-get install libpcl-all
```

## Usage
```
roslaunch freenect_launch freenect.launch
roslaunch darknet_ros darknet_ros_kinect.launch  
```
To use custom-trained weights, some configuration is necessary:

1. Add the configuration file to the `cfg` directory and the weights to the `weights` directory in `./darknet_ros/yolo_network_config`.
2. Add a new YAML file to `./darknet_ros/config`, following the format of existing YAML files.
3. Finally, modify and save the line `<arg name="network_param_file" default="$(find darknet\_ros)/config/yolov2.yaml"/>` in `./darknet_ros/launch/darknet_ros_kinect.launch` to point to your newly created YAML file.

## Node
**`name` darknet\_kinect**  
A node that sends 3D information.

### Subscribe Topic

* **`/camera/depth_registered/points`**   
3D information from freenect (sensor_msgs::PointCloud2).

* **`/darknet_ros/bounding_boxes`**  
Planar coordinate information from darknet_ros (darknet_ros_msgs::BoundingBoxes).

### Publish Topic

* **`/darknet/rgb/image_raw`**  
Color image converted from freenect's point cloud (sensor_msgs::Image).

* **`/darknet_ros/object_position`**  
3D coordinate information combining darknet_ros and freenect's point cloud (darknet_ros_kinect::ObjectPositions).
