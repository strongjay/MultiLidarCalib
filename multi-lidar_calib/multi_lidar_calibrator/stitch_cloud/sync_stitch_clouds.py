#!/usr/bin/env python3

import numpy as np
import rclpy
import ros2_numpy as rnp
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from typing import Dict
from sensor_msgs.msg import PointField 
from ros2_numpy.point_cloud2 import pointcloud2_to_xyzi_array
from message_filters import ApproximateTimeSynchronizer, Subscriber

fields = [
    PointField(name='x', offset=0, count=1, datatype=PointField.FLOAT32),
    PointField(name='y', offset=4, count=1, datatype=PointField.FLOAT32),
    PointField(name='z', offset=8, count=1, datatype=PointField.FLOAT32),
    PointField(name='intensity', offset=12, count=1, datatype=PointField.FLOAT32)
]

dtype = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('intensity', np.float32)
])

class SyncAndStitchCloudsPub(Node):
    def __init__(self, topic_names: list, target_lidar: str, tf_Result: Dict):
        super().__init__('SyncAndStitchCloudsPub')
        self.get_logger().info("Finish multi-lidar-calibration, publishing stitched cloud...")
        if not tf_Result:
            raise ValueError("calibrate transform result is empty!")
            
        self.tf_Result = tf_Result
        self.target_lidar = target_lidar
        self.topic_names = topic_names

        self.needed_frame_ids = []
        for topic in self.topic_names:
            self.needed_frame_ids.append(topic.split('/')[-1] if '/' in topic else topic)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for stitched cloud
        self.stitched_pub = self.create_publisher(PointCloud2, 'stitched_cloud', 10)
        
        # 创建消息缓存字典，用于ApproximateTimeSynchronizer
        self.subscribers = []
        self.sync_subscribers = []
        
        # 为每个话题创建Subscriber
        for topic in topic_names:
            lidar_name = topic.split('/')[-1] if '/' in topic else topic
            sub = Subscriber(self, PointCloud2, topic)
            self.subscribers.append(sub)
            self.sync_subscribers.append(sub)
        
        # 创建ApproximateTimeSynchronizer
        # 设置时间同步容差为100ms (0.1秒)
        self.sync = ApproximateTimeSynchronizer(
            self.sync_subscribers,
            queue_size=10,  # 同步队列大小
            slop=0.1  # 时间同步容差(秒)
        )
        # 注册回调函数
        self.sync.registerCallback(self.synced_pointcloud_callback)
        self.get_logger().info(f"stitch clouds:({self.needed_frame_ids} in {self.target_lidar} coordinate )")
        # Buffer to hold transformed point clouds
        self.transformed_clouds = {}
        
        self.get_logger().info(f"Stitched Cloud Publisher initialized with {len(topic_names)} topics, sync tolerance: 100ms")

    def synced_pointcloud_callback(self, *msgs):
        """同步回调函数，处理时间戳相近的点云消息"""
        try:
            # 确保接收到的消息数量与订阅的话题数量一致
            if len(msgs) != len(self.topic_names):
                self.get_logger().error(f"接收到的消息数量({len(msgs)})与订阅话题数量({len(self.topic_names)})不匹配")
                return
            # 创建消息与话题名称的映射字典
            msg_topic_map = {}

            # 处理每个同步的点云消息
            for msg in msgs:
                frame_id = msg.header.frame_id #.split('/')[-1] if '/' in msg.header.frame_id else msg.header.frame_id
                if(frame_id not in self.needed_frame_ids):
                    self.get_logger().error(f"点云帧id: {frame_id} 不在对齐的点云数据集({self.needed_frame_ids})")
                    continue
                msg_topic_map[frame_id] = msg

            # 检查是否所有话题都有对应的消息
            if len(msg_topic_map) != len(self.needed_frame_ids):
                missing_topics = set(self.needed_frame_ids) - set(msg_topic_map.keys())
                self.get_logger().error(f"缺少某些话题的消息: {missing_topics}")
                return
            
            # 处理每个同步的消息
            for topic_name, msg in msg_topic_map.items():
                # 从topic_name中提取lidar_name
                lidar_name = topic_name#.split('/')[-1] if '/' in topic_name else topic_name
                self.process_synced_message(msg, lidar_name)
            
            # 发布拼接后的点云
            self.publish_stitched_cloud()

        except Exception as e:
            self.get_logger().error(f"Error in synced callback: {str(e)}", throttle_duration_sec=1.0)

    def process_synced_message(self, msg: PointCloud2, lidar_name: str):
        """处理经过时间同步的消息"""
        try:
            if msg is None:
                self.get_logger().error(f"Received None message for {lidar_name}")
                return
                
            # 检查点云字段结构（兼容xyz合并字段）
            required_fields = {'x', 'y', 'z', 'intensity'}
            msg_fields = {field.name for field in msg.fields}
            
            # 标准分离字段 (x,y,z)
            if required_fields.issubset(msg_fields):
                try:
                    xyzi = pointcloud2_to_xyzi_array(msg)
                    if xyzi is None or len(xyzi) == 0:
                        self.get_logger().warn(f"Empty point cloud from {lidar_name}")
                        return
                        
                    xyz = xyzi[:, :3]  # 分离坐标

                    # 目标雷达无需变换，但需要转换为结构化数组格式
                    if lidar_name == self.target_lidar:
                        structured_arr = np.zeros(len(xyzi), dtype=dtype)
                        structured_arr['x'] = xyzi[:, 0]
                        structured_arr['y'] = xyzi[:, 1]
                        structured_arr['z'] = xyzi[:, 2]
                        structured_arr['intensity'] = xyzi[:, 3]
                        self.transformed_clouds[lidar_name] = structured_arr
                        return

                    if lidar_name not in self.tf_Result.keys():
                        self.get_logger().warn(f"No calibration result for {lidar_name}")
                        return
                    
                    # 其他点云应用标定变换
                    tf_matrix = self.tf_Result[lidar_name]
                    homogeneous_xyz = np.column_stack([xyz, np.ones(len(xyz))])
                    transformed_xyz = np.dot(homogeneous_xyz, tf_matrix.T)[:, :3]
                    # 重建含强度值的点云
                    structured_arr = np.zeros(len(xyzi), dtype=dtype)
                    structured_arr['x'] = transformed_xyz[:, 0]
                    structured_arr['y'] = transformed_xyz[:, 1]
                    structured_arr['z'] = transformed_xyz[:, 2]
                    structured_arr['intensity'] = xyzi[:, 3]  # 保留原始强度
                    self.transformed_clouds[lidar_name] = structured_arr
                except Exception as e:
                    self.get_logger().error(f"Error processing point cloud data for {lidar_name}: {str(e)}")
                    return
            # 不支持的格式
            else:
                self.get_logger().error(
                    f"Unsupported point cloud format from {lidar_name}. "
                    f"Required fields: {required_fields}, Got: {msg_fields}"
                )
                return

        except Exception as e:
            self.get_logger().error(
                f"Error processing {lidar_name}: {str(e)}", 
                throttle_duration_sec=1.0
            )
            # 确保在异常情况下不会留下部分处理的数据
            if lidar_name in self.transformed_clouds:
                del self.transformed_clouds[lidar_name]

    def create_transformed_msg(self, original_msg: PointCloud2, xyz: np.ndarray) -> PointCloud2:
        """创建标准化格式的点云消息"""
        # 构造结构化数组
        dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
        structured_arr = np.zeros(len(xyz), dtype=dtype)
        structured_arr['x'] = xyz[:, 0]
        structured_arr['y'] = xyz[:, 1]
        structured_arr['z'] = xyz[:, 2]
        
        # 转换回ROS消息
        msg = rnp.msgify(PointCloud2, structured_arr)
        msg.header = original_msg.header
        msg.header.frame_id = self.target_lidar
        msg.height = 1
        msg.width = len(xyz)
        msg.is_bigendian = False
        msg.point_step = 12  # 3*float32
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        return msg

    def publish_tf(self, lidar_name: str, tf_matrix: np.ndarray):
        """Publish transform from LiDAR to target frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.target_lidar
        t.child_frame_id = f"{lidar_name}_calibrated"
        
        # Extract translation and rotation from transformation matrix
        t.transform.translation.x = tf_matrix[0, 3]
        t.transform.translation.y = tf_matrix[1, 3]
        t.transform.translation.z = tf_matrix[2, 3]
        
        # Convert rotation matrix to quaternion
        from scipy.spatial.transform import Rotation as R
        rotation = R.from_matrix(tf_matrix[:3, :3]).as_quat()
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_stitched_cloud(self):
        """拼接所有对齐后的点云并发布"""
        try:
            if not self.transformed_clouds:
                self.get_logger().warn("No transformed clouds available for stitching")
                return

            # 计算总点数并验证
            total_points = 0
            for cloud in self.transformed_clouds.values():
                total_points += len(cloud)
            
            if total_points == 0:
                self.get_logger().warn("All transformed clouds are empty")
                return

            # 创建输出数组 xyzi格式 (使用结构化数组)
            all_points = np.zeros(total_points, dtype=dtype)
            current_idx = 0
            
            # 填充数组与转换后的点
            for lidar_name, cloud in self.transformed_clouds.items():
                if cloud is None:
                    self.get_logger().warn(f"Cloud from {lidar_name} is None, skipping")
                    continue
                    
                num_points = len(cloud)
                if num_points > 0:
                    try:
                        # 复制所有字段
                        for field in ['x', 'y', 'z', 'intensity']:
                            if field in cloud.dtype.names:
                                all_points[field][current_idx:current_idx+num_points] = cloud[field]
                        current_idx += num_points
                    except Exception as e:
                        self.get_logger().error(f"Error copying points from {lidar_name}: {str(e)}")

            # 确保有点被添加
            if current_idx == 0:
                self.get_logger().warn("No points were successfully added to the stitched cloud")
                return
                
            # 创建输出消息
            stitched_msg = PointCloud2()
            stitched_msg.header.stamp = self.get_clock().now().to_msg()
            stitched_msg.header.frame_id = self.target_lidar  # 修改为target_lidar
            stitched_msg.height = 1
            stitched_msg.width = current_idx
            stitched_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            stitched_msg.is_bigendian = False
            stitched_msg.point_step = 16
            stitched_msg.row_step = stitched_msg.point_step * stitched_msg.width
            stitched_msg.is_dense = True
            
            try:
                stitched_msg.data = all_points[:current_idx].tobytes()
                self.stitched_pub.publish(stitched_msg)
            except Exception as e:
                self.get_logger().error(f"Error converting points to bytes: {str(e)}")
                return
            self.get_logger().info(
                f"Published stitched point cloud with {current_idx} points",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing stitched cloud: {str(e)}", 
                                  throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    # 这些参数应该从calibrator节点获取
    # 示例值，实际应该从参数或配置文件中读取
    topic_names = ['/lidar1', '/lidar2', '/lidar3']  # 输入话题列表
    target_lidar = 'lidar1'  # 目标坐标系
    tf_Result = {
        'lidar1': np.eye(4),  # 示例变换矩阵
        'lidar2': np.eye(4),
        'lidar3': np.eye(4)
    }
    
    node = SyncAndStitchCloudsPub(topic_names, target_lidar, tf_Result)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()