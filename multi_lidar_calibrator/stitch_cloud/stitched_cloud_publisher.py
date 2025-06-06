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
from ros2_numpy.point_cloud2 import split_xyz

class StitchedCloudPublisher(Node):
    def __init__(self, topic_names: list, target_lidar: str, tf_Result: Dict):
        super().__init__('stitched_cloud_publisher')
        
        if not tf_Result:
            raise ValueError("tf_Result cannot be empty")
            
        self.tf_Result = tf_Result
        self.target_lidar = target_lidar
        self.topic_names = topic_names
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for stitched cloud
        self.stitched_pub = self.create_publisher(PointCloud2, 'stitched_cloud', 10)
        
        # Subscribers for each LiDAR
        self.subscribers = []
        for topic in topic_names:
            lidar_name = topic.split('/')[-1] if '/' in topic else topic
            self.subscribers.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, lidar=lidar_name: self.pointcloud_callback(msg, lidar),
                    10
                )
            )
        
        # Buffer to hold transformed point clouds
        self.transformed_clouds = {}
        
        self.get_logger().info("Stitched Cloud Publisher initialized")

    def pointcloud_callback(self, msg: PointCloud2, lidar_name: str):
        """Apply calibration transform and publish stitched point cloud."""
        try:
            if lidar_name == self.target_lidar:
                # 目标雷达无需变换
                new_msg = msg
                new_msg.header.frame_id = self.target_lidar
                self.transformed_clouds[lidar_name] = new_msg
                return

            if lidar_name not in self.tf_Result:
                self.get_logger().warn(f"No calibration result for {lidar_name}")
                return

            # 检查点云字段结构（兼容xyz合并字段）
            required_fields = {'x', 'y', 'z'}
            msg_fields = {field.name for field in msg.fields}
            
            # 情况1：标准分离字段 (x,y,z)
            if required_fields.issubset(msg_fields):
                xyz = split_xyz(rnp.numpify(msg))
                # points = rnp.numpify(msg)
                # xyz = np.column_stack([points['x'], points['y'], points['z']])
                # num_points = len(xyz)
            
            # 情况2：不支持的格式
            else:
                self.get_logger().error(
                    f"Unsupported point cloud format from {lidar_name}. "
                    f"Required fields: {required_fields}, Got: {msg_fields}"
                )
                return

            # 应用标定变换
            tf_matrix = self.tf_Result[lidar_name]
            homogeneous_xyz = np.column_stack([xyz, np.ones(len(xyz))])
            transformed_xyz = np.dot(homogeneous_xyz, tf_matrix.T)[:, :3]

            # 重建标准化点云消息
            new_msg = self.create_transformed_msg(msg, transformed_xyz)
            self.transformed_clouds[lidar_name] = new_msg

        except Exception as e:
            self.get_logger().error(
                f"Error processing {lidar_name}: {str(e)}", 
                throttle_duration_sec=1.0
            )

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
        """Combine all transformed point clouds and publish."""
        try:
            # Validate we have clouds to stitch
            if not self.transformed_clouds:
                self.get_logger().warn("No transformed clouds available for stitching")
                return

            # Calculate total points and validate
            total_points = 0
            for cloud_msg in self.transformed_clouds.values():
                try:
                    # points = rnp.numpify(cloud_msg)
                    # xyz = np.column_stack([points['x'], points['y'], points['z']])
                    # num_points = len(xyz)
                    xyz = split_xyz(rnp.numpify(cloud_msg))
                except Exception as e:
                    self.get_logger().error(f"Error counting points: {str(e)}")
                    continue
            
            if total_points == 0:
                self.get_logger().warn("All transformed clouds are empty")
                return

            # Create output array
            all_points = np.empty((total_points, 3), dtype=np.float32)
            current_idx = 0
            
            # Fill the array with transformed points
            for lidar_name, cloud_msg in self.transformed_clouds.items():
                try:
                    # points = rnp.numpify(cloud_msg)
                    points = split_xyz(rnp.numpify(cloud_msg))
                    if isinstance(points, np.ndarray):
                        if hasattr(points, 'dtype') and hasattr(points.dtype, 'names') and 'xyz' in points.dtype.names:
                            # Structured array with xyz field
                            xyz = points['xyz']
                            num_points = len(xyz)
                        else:
                            # Regular array (from merged xyz field)
                            xyz = points.reshape(-1, 3)
                            num_points = len(xyz)
                        
                        if num_points > 0:
                            all_points[current_idx:current_idx+num_points] = xyz
                            current_idx += num_points
                except Exception as e:
                    self.get_logger().error(f"Error processing {lidar_name} cloud: {str(e)}")
                    continue

            # Create output message
            stitched_msg = PointCloud2()
            stitched_msg.header.stamp = self.get_clock().now().to_msg()
            stitched_msg.header.frame_id = self.target_lidar
            stitched_msg.height = 1
            stitched_msg.width = current_idx
            stitched_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            stitched_msg.is_bigendian = False
            stitched_msg.point_step = 12
            stitched_msg.row_step = stitched_msg.point_step * stitched_msg.width
            stitched_msg.is_dense = True
            stitched_msg.data = all_points[:current_idx].tobytes()

            self.stitched_pub.publish(stitched_msg)
            self.get_logger().info(
                f"Published stitched point cloud with {current_idx} points",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing stitched cloud: {str(e)}", 
                                  throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    
    # These parameters would normally come from the calibrator node
    lidar_dict = {}  # Should be populated with Lidar objects containing tf_matrix
    topic_names = []  # List of input topics
    target_lidar = ""  # Target frame ID
    
    node = StitchedCloudPublisher(lidar_dict, topic_names, target_lidar)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()