#!/bin/bash 
# 直接用命令行调用
python3 multi_lidar_calibrator/multi_lidar_calibrator.py --ros-args --params-file config/demo.yaml

# ros2 package 运行
# colcon build
# source install/setup.bash
# ros2 launch multi_lidar_calibrator calibration.launch.py parameter_file:=config/demo.yaml