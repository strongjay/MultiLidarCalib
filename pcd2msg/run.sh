#!/bin/bash
# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data1/left &
# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data1/right &
# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data1/main

# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data2/lidar1 &
# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data2/lidar2 &
# ./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data2/lidar3

./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data3/main &
./build/pcd2MshPublisher/pcd_publisher --ros-args -p pcd_folder:=data3/via 