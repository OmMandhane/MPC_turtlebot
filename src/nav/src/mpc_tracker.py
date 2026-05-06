#!/usr/bin/env python3
import math
import numpy as np
import casadi as ca

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


def euler_from_quaternion(x, y, z, w):
    """Converts quaternion orientation to Euler yaw."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class MPCTracker(Node):
    """
    Model Predictive Control (MPC) Node for path tracking and dynamic obstacle avoidance.
    Utilizes an Inverse Barrier function for repulsion and a Monotonic Progress Tracker
    to prevent path-jumping in sharp turns.
    """
    def __init__(self):
        super().__init__('mpc_tracker')

        # --- Parameters ---
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('N', 25)

        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.dt = self.get_parameter('dt').value
        self.N = self.get_parameter('N').value

        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.global_path = None
        self.path_received = False
        self._progress_idx = 0
        self.obs_points = []

        # --- Solver Memory (Warm Start) ---
        self.last_u_opt = None
        self.last_x_opt = None

        # --- ROS 2 Interfaces ---
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/path', self.path_cb, qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.horizon_pub = self.create_publisher(Marker, '/mpc_horizon', 10)
        self.obs_pub = self.create_publisher(Marker, '/mpc_obstacle', 10)

        # Control loop timer (Syncs to Sim Time if use_sim_time is true)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("MPC Tracker Initialized Successfully.")

    # -------------------------------------------------------------------------
    # Sensor Callbacks
    # -------------------------------------------------------------------------

    def odom_cb(self, msg):
        """Updates robot state from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_cb(self, msg):
        """Extracts and downsamples obstacle surface points from LiDAR."""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0

        danger = []
        # Downsample: Process every 15th ray to reduce solver load
        for i in range(0, len(ranges), 15):
            d = ranges[i]
            if d < 1.5:  # Danger zone threshold
                angle = msg.angle_min + i * msg.angle_increment
                
                # Transform local polar coordinates to global cartesian
                lx = d * math.cos(angle)
                ly = d * math.sin(angle)
                gx = self.x + lx * math.cos(self.yaw) - ly * math.sin(self.yaw)
                gy = self.y + lx * math.sin(self.yaw) + ly * math.cos(self.yaw)
                danger.append([gx, gy, d])

        # Retain only the 10 closest points to define the obstacle surface
        if danger:
            danger.sort(key=lambda p: p[2])
            self.obs_points = [[p[0], p[1]] for p in danger[:10]]
        else:
            self.obs_points = []

    def path_cb(self, msg):
        """Receives the global path and resets the progress tracker."""
        pts = []
        for pose in msg.poses:
            px = pose.pose.position.x
            py = pose.pose.position.y
            q = pose.pose.orientation
            pyaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
            pts.append([px, py, pyaw])
        
        self.global_path = np.array(pts)
        self.path_received = True
        self._progress_idx = 0 

    # -------------------------------------------------------------------------
    # Trajectory Generation & MPC Solver
    # -------------------------------------------------------------------------

    def get_reference_trajectory(self):
        """Generates a localized target trajectory using monotonic progression."""
        path = self.global_path
        n = len(path)

        # 1. Monotonic Window Search (Prevents Hairpin Path-Jumping)
        search_start = self._progress_idx
        search_end = min(self._progress_idx + 50, n)

        window_dists = np.linalg.norm(path[search_start:search_end, :2] - [self.x, self.y], axis=1)
        local_best = int(np.argmin(window_dists))
        self._progress_idx = search_start + local_best

        # 2. Dynamic Spacing (Matches target point distance to robot velocity)
        pt_dist = np.linalg.norm(path[1, :2] - path[0, :2]) if n > 1 else 0.1
        idx_step = max(1, int(self.v_max * self.dt / pt_dist)) if pt_dist > 1e-4 else 1

        # 3. Build Horizon Array
        ref = np.zeros((3, self.N))
        for i in range(self.N):
            idx = min(self._progress_idx + i * idx_step, n - 1)
            ref[:, i] = path[idx]

        return ref

    def solve_mpc(self, current_state, ref_traj):
        """Formulates and solves the non-linear optimization problem."""
        opti = ca.Opti()
        
        # State and Control Variables
        X = opti.variable(3, self.N + 1)
        U = opti.variable(2, self.N)
        
        # Warm Start: Inject previous solution to prevent saddle-point oscillation
        if self.last_u_opt is not None and self.last_x_opt is not None:
            opti.set_initial(U, self.last_u_opt)
            opti.set_initial(X, self.last_x_opt)
            
        # Tuning Weights
        Q_x = 1.0; Q_y = 1.0; Q_theta = 0.1  # Path tracking adherence
        Q_v = 5.0                            # Cruise control (forward motivation)
        R_w = 0.1                            # Steering penalty
        W_obs = 0.5                          # Obstacle repulsion strength

        cost = 0
        for k in range(self.N):
            # 1. Unicycle Kinematic Constraints
            opti.subject_to(X[0, k+1] == X[0, k] + U[0, k] * ca.cos(X[2, k]) * self.dt)
            opti.subject_to(X[1, k+1] == X[1, k] + U[0, k] * ca.sin(X[2, k]) * self.dt)
            opti.subject_to(X[2, k+1] == X[2, k] + U[1, k] * self.dt)

            # 2. Path Tracking Cost (Includes Cosine Trick for Pi-Crossing)
            cost += Q_x * (X[0, k] - ref_traj[0, k])**2
            cost += Q_y * (X[1, k] - ref_traj[1, k])**2
            cost += Q_theta * 2.0 * (1.0 - ca.cos(X[2, k] - ref_traj[2, k]))
            
            # 3. Actuation Cost (Cruise Control configuration)
            cost += Q_v * (self.v_max - U[0, k])**2 
            cost += R_w * U[1, k]**2

            # 4. Inverse Barrier Obstacle Avoidance
            for obs in self.obs_points:
                dist_sq = ((X[0, k] - obs[0])**2 + (X[1, k] - obs[1])**2)
                cost += W_obs / (dist_sq + 0.001)

        # Apply Boundary Conditions
        opti.subject_to(X[:, 0] == current_state)
        opti.subject_to(opti.bounded(0.0, U[0, :], self.v_max))
        opti.subject_to(opti.bounded(-self.w_max, U[1, :], self.w_max))

        # Solve
        opti.minimize(cost)
        opti.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.sb': 'yes'})
        sol = opti.solve()
        
        # Cache solution for next cycle's Warm Start
        self.last_u_opt = sol.value(U)
        self.last_x_opt = sol.value(X)
        
        return sol.value(U), sol.value(X)

    # -------------------------------------------------------------------------
    # Visualization & Execution
    # -------------------------------------------------------------------------

    def publish_horizon_marker(self, x_horizon):
        """Visualizes the MPC's predicted trajectory in RViz."""
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpc_horizon'; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0
        
        for i in range(self.N + 1):
            p = Point()
            p.x = float(x_horizon[0, i])
            p.y = float(x_horizon[1, i])
            m.points.append(p)
        self.horizon_pub.publish(m)

    def publish_obstacle_marker(self):
        """Visualizes the tracked obstacle surface points in RViz."""
        if not self.obs_points: return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'mpc_obstacle'; m.id = 1
        m.type = Marker.SPHERE_LIST; m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color.r = 1.0; m.color.a = 0.8
        
        for obs in self.obs_points:
            p = Point()
            p.x = float(obs[0]); p.y = float(obs[1]); p.z = 0.15
            m.points.append(p)
        self.obs_pub.publish(m)

    def control_loop(self):
        """Main timer loop executing the MPC pipeline."""
        if not self.path_received: return

        ref_traj = self.get_reference_trajectory()
        dist_to_goal = math.hypot(self.x - self.global_path[-1, 0], self.y - self.global_path[-1, 1])
        cmd = Twist()
        
        if dist_to_goal < 0.2:
            self.get_logger().info("Goal reached!")
            self.cmd_pub.publish(cmd)
            return

        try:
            u_opt, x_horizon = self.solve_mpc([self.x, self.y, self.yaw], ref_traj)
            
            cmd.linear.x = float(u_opt[0, 0])
            cmd.angular.z = float(u_opt[1, 0])
            self.cmd_pub.publish(cmd)
            
            self.publish_horizon_marker(x_horizon)
            self.publish_obstacle_marker()
            
        except Exception as e:
            self.get_logger().error(f"MPC failed: {e}")
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()