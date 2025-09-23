#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np
import math

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trajectory_sub = self.create_subscription(
            Path, '/trajectory', self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.declare_parameters(namespace='', parameters=[
            ('lookahead_distance', 0.3),
            ('max_linear_velocity', 0.5),
            ('max_angular_velocity', 0.5),
            ('position_tolerance', 0.1),
            ('orientation_tolerance', 0.1),
            ('kp_linear', 1.0),
            ('ki_linear', 0.0),
            ('kd_linear', 0.1),
            ('kp_angular', 2.0),
            ('ki_angular', 0.0),
            ('kd_angular', 0.1),
        ])
        
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.orient_tolerance = self.get_parameter('orientation_tolerance').value
        
        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        
        self.current_pose = None
        self.trajectory = None
        self.trajectory_start_time = None
        self.current_target_idx = 0
        self.trajectory_complete = False
        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.prev_time = None
        
        self.control_timer = self.create_timer(0.05, self.control_callback) 
        
        self.get_logger().info("Trajectory Controller initialized")
    
    def trajectory_callback(self, msg):
        self.trajectory = msg
        self.trajectory_start_time = self.get_clock().now()
        self.current_target_idx = 0
        self.trajectory_complete = False
        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.prev_time = None
        
        self.get_logger().info(f"Received new trajectory with {len(msg.poses)} points")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def find_target_point(self):
        if not self.trajectory or not self.current_pose:
            return None, -1
        
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])
        
        min_dist = float('inf')
        closest_idx = self.current_target_idx
        
        for i in range(self.current_target_idx, len(self.trajectory.poses)):
            traj_pos = np.array([
                self.trajectory.poses[i].pose.position.x,
                self.trajectory.poses[i].pose.position.y
            ])
            
            dist = np.linalg.norm(current_pos - traj_pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        target_idx = closest_idx
        for i in range(closest_idx, len(self.trajectory.poses)):
            traj_pos = np.array([
                self.trajectory.poses[i].pose.position.x,
                self.trajectory.poses[i].pose.position.y
            ])
            
            dist = np.linalg.norm(current_pos - traj_pos)
            if dist >= self.lookahead_distance:
                target_idx = i
                break
            target_idx = i
        
        self.current_target_idx = max(closest_idx, self.current_target_idx)
        
        return self.trajectory.poses[target_idx], target_idx
    
    def compute_control_commands(self, target_pose):
        if not self.current_pose or not target_pose:
            return 0.0, 0.0
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_quat = self.current_pose.orientation
        _, _, current_yaw = euler_from_quaternion([
            current_quat.x, current_quat.y, current_quat.z, current_quat.w
        ])
        
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        desired_yaw = math.atan2(dy, dx)
        angular_error = self.normalize_angle(desired_yaw - current_yaw)
        
        current_time = self.get_clock().now()
        if self.prev_time is None:
            dt = 0.05 
        else:
            dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        
        linear_derivative = (distance_error - self.prev_linear_error) / dt if dt > 0 else 0.0
        self.linear_error_sum += distance_error * dt
        
        linear_vel = (self.kp_linear * distance_error + 
                     self.ki_linear * self.linear_error_sum +
                     self.kd_linear * linear_derivative)
        
        angular_derivative = (angular_error - self.prev_angular_error) / dt if dt > 0 else 0.0
        self.angular_error_sum += angular_error * dt
        
        angular_vel = (self.kp_angular * angular_error +
                      self.ki_angular * self.angular_error_sum +
                      self.kd_angular * angular_derivative)
        
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        if abs(angular_error) > 0.5: 
            linear_vel *= 0.5
        
        self.prev_linear_error = distance_error
        self.prev_angular_error = angular_error
        
        return linear_vel, angular_vel
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def is_trajectory_complete(self):
        if not self.trajectory or not self.current_pose:
            return False
        
        if self.current_target_idx >= len(self.trajectory.poses) - 1:
            final_pose = self.trajectory.poses[-1]
            dx = final_pose.pose.position.x - self.current_pose.position.x
            dy = final_pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            return distance < self.pos_tolerance
        
        return False
    
    def control_callback(self):
        if self.trajectory_complete or not self.trajectory or not self.current_pose:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        if self.is_trajectory_complete():
            self.trajectory_complete = True
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Trajectory completed!")
            return
        
        target_pose, target_idx = self.find_target_point()
        
        if target_pose is None:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        linear_vel, angular_vel = self.compute_control_commands(target_pose)
        
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        if self.current_target_idx % 10 == 0: 
            self.get_logger().debug(
                f"Tracking point {target_idx}/{len(self.trajectory.poses)-1}, "
                f"cmd_vel: linear={linear_vel:.3f}, angular={angular_vel:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    controller_node = TrajectoryController()
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()