#!/bin/bash 
# 直接用命令行调用
python3 multi_lidar_calibrator/multi_lidar_calibrator.py --ros-args --params-file config/params3.yaml

# ros2 package 运行
# colcon build
# source install/setup.bash
# ros2 launch multi_lidar_calibrator calibration.launch.py parameter_file:=config/demo.yaml


# config/param1.yaml
# 就是ros2 的参数文件结构
# 默认采用pcd文件标定
#   需要指定文件夹路径，目标点云要和文件名一致
# 也可以采用监听话题标定
#   需要指定标定点云topic、目标点云topic、点云的frame_id要和topic名称一致、初始位姿（默认为0，key为topic名称）