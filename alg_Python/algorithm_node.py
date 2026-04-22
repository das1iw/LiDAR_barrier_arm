#!/usr/bin/env python3
"""
ROS2 Node - Simple barrier arm detection
Publishes the thinnest and longest cluster from LiDAR data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String, Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import struct
import sys
import os
from collections import deque

# Add paths
sys.path.insert(0, '/root/Desktop/arm_detection')

from reader import read_points
from preprocess import crop_roi
from clustering import cluster
from geometry import fit_line_pca, compute_angle

class LidarProcessingNode(Node):
    def __init__(self, bag_path):
        super().__init__('lidar_processing_node')
        
        # Publishers
        self.raw_pub = self.create_publisher(PointCloud2, '/lidar_raw', 10)
        self.filtered_pub = self.create_publisher(PointCloud2, '/lidar_filtered', 10)
        self.clusters_pub = self.create_publisher(MarkerArray, '/lidar_clusters', 10)
        self.direction_pub = self.create_publisher(Marker, '/barrier_direction', 10)
        self.state_pub = self.create_publisher(String, '/barrier_state', 10)
        self.angle_pub = self.create_publisher(Float32, '/barrier_angle', 10)
        
        self.frame_idx = 0
        
        self.get_logger().info(f'Loading data from: {bag_path}')
        
        # Load data
        try:
            from reader import read_points
            self.frames = list(read_points(bag_path))
            self.get_logger().info(f'Successfully loaded {len(self.frames)} frames')
            
            # Timer for publishing (10 Hz)
            self.timer = self.create_timer(0.1, self.process_and_publish)
        except Exception as e:
            self.get_logger().error(f'Error loading data: {e}')
            self.frames = None
    
    def points_to_pointcloud2(self, points, frame_id="lidar_frame3"):
        """Convert points to PointCloud2 message"""
        if points is None or len(points) == 0:
            return None
        
        if isinstance(points, np.ndarray) and points.dtype.names is not None:
            x = points['x'].astype(np.float32)
            y = points['y'].astype(np.float32)
            z = points['z'].astype(np.float32)
            intensity = points['intensity'].astype(np.float32) if 'intensity' in points.dtype.names else np.ones(len(x), dtype=np.float32)
        else:
            pts = points
            if pts.ndim == 1:
                pts = np.array([(p[0], p[1], p[2]) for p in pts])
            
            if len(pts.shape) == 2 and pts.shape[1] >= 3:
                x = pts[:, 0].astype(np.float32)
                y = pts[:, 1].astype(np.float32)
                z = pts[:, 2].astype(np.float32)
                intensity = np.ones(len(x), dtype=np.float32)
            else:
                return None
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.height = 1
        msg.width = len(x)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
                
        points_np = np.column_stack((x, y, z, intensity)).astype(np.float32)
        msg.data = points_np.tobytes()
        
        return msg
    
    def get_cluster_features(self, cluster):
        """Calculate cluster features: length, width, height, L/W ratio, linearity"""
        pts = cluster
        if len(pts) < 5:
            return 0, 0, 0, 0, 0

        xyz = pts[:, :3]
        centroid = xyz.mean(axis=0)
        centered = xyz - centroid

        
        
        n = centered.shape[0]
        cov = (centered.T @ centered) / (n-1) # cov = np.cov(centered.T)
        
        
        
        eigvals, eigvecs = np.linalg.eigh(cov)
        idx = np.argsort(eigvals)[::-1]
        eigvals = eigvals[idx]
        eigvecs = eigvecs[:, idx]

        # Stabilize eigenvalues
        eigvals[1] = max(eigvals[1], 1e-3)

        # Linearity
        linearity = eigvals[0] / (max(eigvals[1], 1e-6) + 1e-8)

        # Length (principal axis)
        projections = centered @ eigvecs[:, 0]
        length = projections.max() - projections.min()

        # Width (second axis)
        width_projections = centered @ eigvecs[:, 1]
        width = width_projections.max() - width_projections.min()

        # Height
        height = xyz[:, 2].max() - xyz[:, 2].min()

        lw_ratio = length / (width + 1e-6)

        return length, width, height, lw_ratio, linearity, eigvecs[:, 0]
    
    def get_cluster_distance(self, cluster):
        """Calculate distance to cluster centroid"""
        cluster_array = cluster
        center = cluster_array.mean(axis=0)
        distance = np.sqrt(center[0]**2 + center[1]**2 + center[2]**2)
        return distance, center
    
    def select_best_cluster(self, clusters):
        """
        Select the thinnest and longest cluster
        Uses scoring: higher score = thinner AND longer
        """
        if not clusters:
            return None, 0, np.zeros(3), (0, 0, 0, 0, 0)
        
        best_cluster = None
        best_score = -1
        best_distance = 0
        best_center = np.zeros(3)
        best_features = (0, 0, 0, 0, 0, np.zeros(3))
        
        for cluster_points in clusters:
            if not isinstance(cluster_points, np.ndarray):
            	cluster_points = np.asarray(cluster_points)
            
            if len(cluster_points) < 10:  # Minimum points for stability
                continue
            
            length, width, height, lw_ratio, linearity, direction = self.get_cluster_features(cluster_points)
            
            # Hard constraints for barrier arm

            if width > 0.5:
                continue
            
            if lw_ratio < 1.0:
                continue
            
            
            # Skip if width is zero or invalid
            width = max(width, 1e-3)
            
            # Score based on length and thinness
            score = length / (width + 1e-6) + 0.5 * length
            
            distance, center = self.get_cluster_distance(cluster_points)
            
            if score > best_score:
                best_score = score
                best_cluster = cluster_points
                best_distance = distance
                best_center = center
                best_features = (length, width, height, lw_ratio, linearity, direction)
        
        return best_cluster, best_distance, best_center, best_features
    
    def cluster_to_markers(self, cluster, distance, features, frame_id="lidar_frame3"):
        """Convert single cluster to MarkerArray for visualization"""
        marker_array = MarkerArray()
        
        if cluster is None:
            return marker_array
        
        cluster_array = cluster
        center = cluster_array.mean(axis=0)
        
        length, width, height, lw_ratio, linearity, _ = features
        
        # Main bounding box marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "best_cluster"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        x_range = cluster_array[:, 0].max() - cluster_array[:, 0].min()
        y_range = cluster_array[:, 1].max() - cluster_array[:, 1].min()
        z_range = cluster_array[:, 2].max() - cluster_array[:, 2].min()
        
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.scale.x = float(max(x_range, 0.3))
        marker.scale.y = float(max(y_range, 0.3))
        marker.scale.z = float(max(z_range, 0.2))
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        marker_array.markers.append(marker)
        
        # Text label
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "cluster_labels"
        text_marker.id = 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(center[0])
        text_marker.pose.position.y = float(center[1])
        text_marker.pose.position.z = float(center[2] + 0.4)
        text_marker.scale.z = 0.35
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.text = f"ARM ({len(cluster)} pts, L={length:.2f}m, W={width:.2f}m, L/W={lw_ratio:.1f}, D={distance:.1f}m)"
        text_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        marker_array.markers.append(text_marker)
        
        return marker_array
    
    def create_direction_marker(self, points, direction, centroid, frame_id="lidar_frame3"):
        """Create arrow marker for cluster orientation"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Get actual length for arrow scaling
        length, _, _, _, _, _ = self.get_cluster_features(points)
        arrow_length = max(0.5, min(2.0, length))

        start = Point()
        start.x = float(centroid[0])
        start.y = float(centroid[1])
        start.z = float(centroid[2])

        end = Point()
        end.x = float(centroid[0] + direction[0] * arrow_length)
        end.y = float(centroid[1] + direction[1] * arrow_length)
        end.z = float(centroid[2])

        marker.points = [start, end]
        marker.scale.x = 0.08
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        return marker
    
    def process_and_publish(self):
        """Main processing loop"""
        if self.frames is None:
            return
        
        if self.frame_idx >= len(self.frames):
            self.frame_idx = 0
            self.get_logger().info('Loop completed, starting over')
        
        points = self.frames[self.frame_idx]
        
        # Publish raw points
        raw_msg = self.points_to_pointcloud2(points, "lidar_frame3")
        if raw_msg:
            self.raw_pub.publish(raw_msg)
        
        try:
            # Crop ROI
            filtered = crop_roi(points)
            
            filtered_msg = self.points_to_pointcloud2(filtered, "lidar_frame3")
            if filtered_msg:
                self.filtered_pub.publish(filtered_msg)
            
            # Clustering
            clusters = cluster(filtered)
            
            print(f"\n{'='*60}")
            print(f"FRAME {self.frame_idx}:")
            print(f"  Total clusters: {len(clusters) if clusters else 0}")
            
            # Select best cluster (thinnest AND longest)
            best_cluster, best_distance, best_center, best_features = self.select_best_cluster(clusters)
            
            if best_cluster is not None:
                length, width, height, lw_ratio, linearity, direction = best_features
                
                print(f"  BEST CLUSTER: {len(best_cluster)} pts")
                print(f"    Length: {length:.2f}m")
                print(f"    Width: {width:.2f}m")
                print(f"    L/W ratio: {lw_ratio:.1f}")
                print(f"    Linearity: {linearity:.2f}")
                print(f"    Distance: {best_distance:.2f}m")
                
                # Publish cluster visualization
                markers = self.cluster_to_markers(best_cluster, best_distance, best_features)
                if markers:
                    self.clusters_pub.publish(markers)
                
                angle = compute_angle(direction)
                
                # Publish direction arrow
                direction_marker = self.create_direction_marker(best_cluster, direction, best_center)
                if direction_marker:
                    self.direction_pub.publish(direction_marker)
                
                # Publish angle
                angle_msg = Float32()
                angle_msg.data = float(angle)
                self.angle_pub.publish(angle_msg)
                
                # Classification
                state_str = "OPEN" if angle >= 80.0 else "CLOSED"
                
                state_msg = String()
                state_msg.data = state_str
                self.state_pub.publish(state_msg)
                
                print(f"  RESULT: Angle={angle:.1f}°, State={state_str}")
                self.get_logger().info(f'Frame {self.frame_idx}: {len(best_cluster)} pts, L={length:.2f}m, W={width:.2f}m, Angle={angle:.1f}°')
            else:
                # No valid cluster found
                self.clusters_pub.publish(MarkerArray())
                state_msg = String()
                state_msg.data = "NO_BARRIER_ARM"
                self.state_pub.publish(state_msg)
                print(f"  RESULT: No suitable cluster found")
            
            print(f"{'='*60}\n")
                
        except Exception as e:
            self.get_logger().warn(f'Error: {e}')
            import traceback
            traceback.print_exc()
        
        self.frame_idx += 1

def main(args=None):
    bag_path = "/root/Desktop/rosbag2_lidar_data"
    
    if not os.path.exists(bag_path):
        print(f"Error: Path {bag_path} does not exist!")
        return
    
    rclpy.init(args=args)
    node = LidarProcessingNode(bag_path)
    
    if node.frames:
        print("=" * 60)
        print("BARRIER ARM DETECTION - SIMPLE MODE")
        print("=" * 60)
        print("Strategy: Publish the thinnest AND longest cluster")
        print("Score = length/width + 0.5*length")
        print("")
        print("This rewards clusters that are:")
        print("  - THIN (small width)")
        print("  - LONG (large length)")
        print("=" * 60)
        print("\nPress Ctrl+C to stop\n")
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\nStopping node")
    else:
        print("Failed to load data")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    