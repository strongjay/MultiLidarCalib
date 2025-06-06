#!/usr/bin/env python3

import numpy as np
import rclpy
import ros2_numpy as rnp
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from typing import Dict

class StitchedCloudPublisher(Node):
    def __init__(self, lidar_dict: Dict, topic_names: list, target_lidar: str):
        super().__init__('stitched_cloud_publisher')
        
        self.lidar_dict = lidar_dict
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
            if lidar_name in self.lidar_dict:
                # Convert ROS PointCloud2 to numpy array
                points = rnp.numpify(msg)
                if 'xyz' not in points.dtype.names:
                    self.get_logger().warn(f"Point cloud from {lidar_name} missing 'xyz' field")
                    return
                
                xyz = points["xyz"]
                if len(xyz) == 0:
                    return
                
                # Apply calibration transform
                tf_matrix = self.lidar_dict[lidar_name].tf_matrix.matrix
                homogeneous_xyz = np.column_stack([xyz, np.ones(len(xyz))])
                transformed_xyz = np.dot(homogeneous_xyz, tf_matrix.T)[:, :3]
                
                # Convert back to PointCloud2
                new_points = np.zeros(len(xyz), dtype=[("xyz", np.float32, 3)])
                new_points["xyz"] = transformed_xyz
                new_msg = rnp.msgify(PointCloud2, new_points)
                
                # Preserve original header info but update frame_id to target frame
                new_msg.header = msg.header
                new_msg.header.frame_id = self.target_lidar
                
                # Store transformed cloud
                self.transformed_clouds[lidar_name] = new_msg
                
                # Publish TF for visualization
                self.publish_tf(lidar_name, tf_matrix)
                
                # Publish stitched cloud if we have all transformed clouds
                if len(self.transformed_clouds) == len(self.topic_names):
                    self.publish_stitched_cloud()
                    
        except Exception as e:
            self.get_logger().error(f"Error processing {lidar_name} point cloud: {str(e)}")

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
            # Combine all point clouds
            combined_points = []
            for lidar_name, cloud_msg in self.transformed_clouds.items():
                points = rnp.numpify(cloud_msg)
                combined_points.append(points["xyz"])
            
            if not combined_points:
                return
                
            # Create new PointCloud2 message
            all_points = np.concatenate(combined_points)
            new_points = np.zeros(len(all_points), dtype=[("xyz", np.float32, 3)])
            new_points["xyz"] = all_points
            
            stitched_msg = rnp.msgify(PointCloud2, new_points)
            stitched_msg.header.stamp = self.get_clock().now().to_msg()
            stitched_msg.header.frame_id = self.target_lidar
            
            self.stitched_pub.publish(stitched_msg)
            self.get_logger().info("Published stitched point cloud", throttle_duration_sec=1.0)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing stitched cloud: {str(e)}")

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