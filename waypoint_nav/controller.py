#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

def quaternion_to_yaw(qx, qy, qz, qw):

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller_fixed')

        self.declare_parameter('control_rate', 20.0)           
        self.declare_parameter('lookahead_dist', 0.35)        
        self.declare_parameter('max_linear_speed', 0.20)      
        self.declare_parameter('max_angular_speed', 2.5)       
        self.declare_parameter('k_linear', 0.8)                
        self.declare_parameter('k_angular', 2.0)               
        self.declare_parameter('goal_threshold', 0.10)         
        self.declare_parameter('yaw_stop_threshold', 0.6)      
        self.declare_parameter('min_move_dist', 0.02)         

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.goal_threshold = float(self.get_parameter('goal_threshold').value)
        self.yaw_stop_threshold = float(self.get_parameter('yaw_stop_threshold').value)
        self.min_move_dist = float(self.get_parameter('min_move_dist').value)

        self.odom_pose = None      
        self.traj_points = None     

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/trajectory', self.trajectory_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Pure Pursuit Controller (fixed) initialized.')

    def odom_cb(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_pose = (x, y, yaw)

    def trajectory_cb(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty /trajectory')
            return
        pts = []
        for p in msg.poses:
            pts.append([float(p.pose.position.x), float(p.pose.position.y)])
        self.traj_points = np.array(pts)
        self.get_logger().info(f'Received trajectory with {len(self.traj_points)} points')

    def control_loop(self):
        if self.odom_pose is None or self.traj_points is None or len(self.traj_points) == 0:
            return

        rx, ry, ryaw = self.odom_pose

        gx, gy = float(self.traj_points[-1, 0]), float(self.traj_points[-1, 1])
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal <= self.goal_threshold:
            self.get_logger().info('Goal reached. Stopping robot.')
            self._publish_stop()
            return

        deltas = self.traj_points - np.array([rx, ry])  
        dists = np.linalg.norm(deltas, axis=1)
        angles_to_pts = np.arctan2(deltas[:,1], deltas[:,0]) 
        angle_diffs = np.array([self._angle_diff(a, ryaw) for a in angles_to_pts])  

        front_mask = (np.abs(angle_diffs) <= (math.pi/2)) & (dists > self.min_move_dist)
        idxs_front = np.where(front_mask)[0]

        if idxs_front.size > 0:
            start_idx = int(idxs_front[0])
        else:
            idxs_far = np.where(dists > self.min_move_dist)[0]
            if idxs_far.size == 0:
                self._publish_stop()
                return
            start_idx = int(idxs_far[0])

        lookahead_idx = self._advance_to_lookahead(start_idx)

        tx = float(self.traj_points[lookahead_idx, 0])
        ty = float(self.traj_points[lookahead_idx, 1])

        dx = tx - rx
        dy = ty - ry
        target_dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._angle_diff(target_yaw, ryaw)


        if abs(yaw_error) > self.yaw_stop_threshold:
            linear_speed = 0.0
        else:
            linear_speed = self.k_linear * target_dist
            linear_speed = max(0.0, min(linear_speed, self.max_linear_speed))

        angular_speed = self.k_angular * yaw_error
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        twist = Twist()
        twist.linear.x = float(linear_speed)
        twist.angular.z = float(angular_speed)
        self.cmd_pub.publish(twist)

        self.get_logger().debug(
            f"start_idx={start_idx} lookahead_idx={lookahead_idx} tx={tx:.2f},{ty:.2f} "
            f"dist={target_dist:.3f} yaw_err={yaw_error:.3f} lin={linear_speed:.3f} ang={angular_speed:.3f}"
        )

    def _advance_to_lookahead(self, start_idx):
        N = len(self.traj_points)
        if start_idx >= N:
            return N - 1
        acc = 0.0
        prev = self.traj_points[start_idx]
        for i in range(start_idx + 1, N):
            p = self.traj_points[i]
            seg = np.linalg.norm(p - prev)
            acc += seg
            prev = p
            if acc >= self.lookahead_dist:
                return i
        return N - 1

    def _angle_diff(self, a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
