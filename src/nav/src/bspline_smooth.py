#!/usr/bin/env python3
"""
Node for generating a mathematically smooth reference path from sparse waypoints.
Utilizes SciPy's B-spline interpolation to create a C2 continuous trajectory 
with precise tangent-based heading calculations.
"""

import csv
import math
import numpy as np
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PythonPathSmoother(Node):
    """
    Reads discrete X/Y waypoints from a CSV file, applies a parametric cubic B-spline 
    to generate a dense, smooth path, and calculates corresponding yaw orientations 
    using the first derivative of the spline.
    """
    def __init__(self):
        super().__init__('python_smoothing_node')

        # --- Parameters ---
        self.declare_parameter('path_file', 'src/nav/waypoints/waypoint.csv')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('num_points', 500)
        
        # Smoothing factor: 0.0 forces the curve through all waypoints exactly.
        # Higher values create a looser approximation.
        self.declare_parameter('smoothing', 0.0)

        # --- Publisher Configuration ---
        # Transient Local durability ensures late-joining subscribers (like the MPC)
        # still receive the path message even if they spin up after it is published.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, 'path', qos)

        # Process and publish the path immediately upon initialization
        self.process_and_publish()

    def process_and_publish(self):
        """Executes the complete pipeline: Ingestion, Sanitization, Spline Fitting, and Publishing."""
        file_path = self.get_parameter('path_file').value
        frame_id = self.get_parameter('frame_id').value
        num_points = self.get_parameter('num_points').value
        smoothing = self.get_parameter('smoothing').value

        try:
            # ---------------------------------------------------------
            # Step 1: Data Ingestion
            # ---------------------------------------------------------
            wx, wy = [], []
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue
                    try:
                        wx.append(float(row[0]))
                        wy.append(float(row[1]))
                    except ValueError:
                        # Gracefully skip string headers
                        continue

            if len(wx) < 4:
                self.get_logger().error(f"Insufficient waypoints (min 4). Got {len(wx)} in {file_path}.")
                return

            # ---------------------------------------------------------
            # Step 2: Data Sanitization
            # ---------------------------------------------------------
            # Identical consecutive points cause singular matrices in SciPy's splprep.
            # Calculate distance between sequential points and filter out duplicates.
            pts = np.column_stack([wx, wy])
            diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            keep_mask = np.concatenate([[True], diffs > 1e-6])
            
            pts = pts[keep_mask]
            wx, wy = pts[:, 0].tolist(), pts[:, 1].tolist()

            self.get_logger().info(f"Loaded {len(wx)} valid waypoints from {file_path}.")

            # ---------------------------------------------------------
            # Step 3: B-Spline Generation
            # ---------------------------------------------------------
            # Fit a parametric cubic B-spline (k=3) to the spatial coordinates.
            tck, _ = splprep([wx, wy], s=smoothing, k=min(3, len(wx) - 1))
            
            # Generate a dense array of parametric parameter 'u' (0.0 to 1.0)
            u_new = np.linspace(0.0, 1.0, num_points)
            
            # Evaluate the spline at 'u' to get the smooth spatial coordinates
            sx, sy = splev(u_new, tck)

            # Evaluate the first derivative of the spline to extract tangent vectors
            dx, dy = splev(u_new, tck, der=1)

            # ---------------------------------------------------------
            # Step 4: Path Construction & Publishing
            # ---------------------------------------------------------
            path_msg = Path()
            path_msg.header.frame_id = frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for x, y, hdx, hdy in zip(sx, sy, dx, dy):
                pose = PoseStamped()
                pose.header = path_msg.header
                
                # Assign spatial position
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = 0.0

                # Calculate kinematic heading (yaw) from the tangent vector
                yaw = float(np.arctan2(hdy, hdx))
                
                # Convert Euler yaw to Quaternion
                pose.pose.orientation.z = float(math.sin(yaw / 2.0))
                pose.pose.orientation.w = float(math.cos(yaw / 2.0))

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published C2-continuous B-Spline path with {num_points} nodes.")

        except FileNotFoundError:
            self.get_logger().error(f"Failed to locate waypoint CSV: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Spline generation failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PythonPathSmoother()
    
    # Maintain node execution to sustain the Transient Local publisher
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()