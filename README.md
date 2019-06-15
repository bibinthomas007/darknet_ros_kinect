# darknet_ros
## Overview
darknetとkienctを組み合わせて物体の立体座標を取得するパッケージ

## Install
```
$ git clone --recursive https://github.com/rionehome/darknet_ros.git
$ cd ../../
$ catkin_make
```
3D情報を取得するために`freenect`とPointCloudライブラリ(PCL)が必須である。

PCLの導入は多分これ↓
```
$ sudo apt-get update
$ sudo apt-get install libpcl-all
```

## Usage
```
roslaunch freenect_launch freenect.launch
roslaunch darknet_ros darknet_ros_kinect.launch  
```
なお、独自で学習させた重みを使うには若干の設定が必要。  

1. まず`./darknet_ros/yolo_network_config`の中にある`cfg` に設定ファイルを、`weights`ディレクトリに重みを追加する。
2. 次に`./darknet_ros/config`にyamlファイルを新規追加する。書式に関してはすでに存在しているyamlファイルを参考に。  
3. 最後に`./darknet_ros/launch`の`darknet_ros_kinect.launch`に記述されている以下の一行の中の`yolov2.yaml`を任意に書き換えて保存すれば完了。  
`<arg name="network_param_file"         default="$(find darknet\_ros)/config/yolov2.yaml"/>`

## Node
**`name` darknet\_kinect**  
3D情報を送信するノード

### Subscribe Topic

* **`/camera/depth_registered/points`**   
freenectからの3D情報（ sensor_msgs::PointCloud2 ）

* **`/darknet_ros/bounding_boxes`**  
darknet_rosからの平面座標情報（ darknet_ros_msgs::BoundingBoxes ）

### Publish Topic

* **`/darknet/rgb/image_raw`**  
freenectの point cloud から変換した color image ( sensor_msgs::Image )

* **`/darknet_ros/object_position`**  
darknet_ros と freenectのpoint cloud を合わせた立体座標情報（ darknet_ros_kinect::ObjectPositions ）
