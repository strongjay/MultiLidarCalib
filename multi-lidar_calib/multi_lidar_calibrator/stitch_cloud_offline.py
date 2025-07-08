
import numpy as np
import rclpy
from rclpy.node import Node
from typing import Dict
from stitch_cloud.sync_stitch_clouds import SyncAndStitchCloudsPub,load_tf_result_from_file

def main(args=None):
    rclpy.init(args=args)
    topic_names = ['/main', '/left', '/right']  # 输入话题列表
    target_lidar = 'main'  # 目标坐标系（与result.txt一致）
    result_file = '/home/work/AWorkSpace/Calibrate_tools/MultiLidarCalib/multi-lidar_calib/baojun/results.txt' 

    # 读取标定结果
    tf_Result = load_tf_result_from_file(result_file, target_lidar)
    print(f"Loaded calibration results: {list(tf_Result.keys())}")

    node = SyncAndStitchCloudsPub(topic_names, target_lidar, tf_Result)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()