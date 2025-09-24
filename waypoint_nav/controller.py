#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np
import math
import traceback

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        #Publisher to command robot velocity. Many controllers subscribe to /cmd_vel.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #Subscribe to the smoothed path (Path message) and robot odometry
        #Path -> produced by PathSmoother. Odometry -> current robot pose.
        self.trajectory_sub = self.create_subscription(
            Path, '/modified_trajectory', self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        #Tunable parameters. Declared here so they can be overridden via ros2 param.
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
        
        #Read parameters into local attributes (easy to reference)
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.pos_tolerance = self.get_parameter('position_tolerance').value
        self.orient_tolerance = self.get_parameter('orientation_tolerance').value
        
        #PID gains for linear and angular controllers
        self.kp_linear = self.get_parameter('kp_linear').value
        self.ki_linear = self.get_parameter('ki_linear').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.ki_angular = self.get_parameter('ki_angular').value
        self.kd_angular = self.get_parameter('kd_angular').value
        
        #State variables updated by subscriptions
        self.current_pose = None          # latest geometry_msgs/Pose
        self.trajectory = None            # latest nav_msgs/Path
        self.trajectory_start_time = None
        self.current_target_idx = 0       # index into trajectory.poses we're tracking
        self.trajectory_complete = False

        #PID integrator/derivative helpers
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.prev_time = None

        #Controller loop runs at 20 Hz (0.05 s). Keeps commands responsive.
        self.control_timer = self.create_timer(0.05, self.control_callback) 
        self.get_logger().info("Trajectory Controller initialized")
    

    def trajectory_callback(self, msg):
        """Receive a new trajectory (Path). Reset tracking state + PID integrators."""
        if msg is None or len(msg.poses) == 0:
            self.get_logger().warn("Received empty trajectory, ignoring.")
            self.trajectory = None
            return

        #Save trajectory and reset tracking indices/state
        self.trajectory = msg
        self.trajectory_start_time = self.get_clock().now()
        self.current_target_idx = 0
        self.trajectory_complete = False
        
        #Reset PID history on new trajectory
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.prev_time = None
        
        self.get_logger().info(f"Received new trajectory with {len(msg.poses)} points")
    

    def odom_callback(self, msg):
        """Update the robot pose from odometry messages."""
        if msg is None:
            self.get_logger().warn("Received None odometry message.")
            return
        #We store the Pose object directly for simple access in control loop
        self.current_pose = msg.pose.pose

    def find_target_point(self):
        """
        Select a target point from the trajectory using a lookahead distance strategy:
        1. Find the closest point forward of the previously tracked index.
        2. Advance until a point at least lookahead_distance away is found.
        Returns: (PoseStamped target_pose, index) or (None, -1) on failure.
        """
        try:
            if not self.trajectory or not self.current_pose or len(self.trajectory.poses) == 0:
                return None, -1
            
            current_pos = np.array([
                self.current_pose.position.x,
                self.current_pose.position.y
            ])
            
            #Find closest point starting from current_target_idx to avoid jumping backwards
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
            
            #From the closest point, search forward to find a point at lookahead distance
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
                target_idx = i  #if none >= lookahead, target the last checked point
            
            #Keep current_target_idx monotonic (do not jump backwards)
            self.current_target_idx = max(closest_idx, self.current_target_idx)
            return self.trajectory.poses[target_idx], target_idx
        except Exception as e:
            #Robust error handling so a single bad trajectory doesn't crash the node
            self.get_logger().error(f"Error in find_target_point: {e}\n{traceback.format_exc()}")
            return None, -1
    

    def compute_control_commands(self, target_pose):
        """
        Compute linear and angular velocities to drive the robot toward target_pose.
        Uses two independent PID controllers:
          - linear: controls forward velocity proportional to distance error
          - angular: controls heading (yaw) to point at the target
        Returns (linear_vel, angular_vel)
        """
        try:
            if not self.current_pose or not target_pose:
                return 0.0, 0.0
            
            #Current robot position & yaw
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_quat = self.current_pose.orientation
            _, _, current_yaw = euler_from_quaternion([
                current_quat.x, current_quat.y, current_quat.z, current_quat.w
            ])
            
            #Target position
            target_x = target_pose.pose.position.x
            target_y = target_pose.pose.position.y
            
            #Distance error (Euclidean)
            dx = target_x - current_x
            dy = target_y - current_y
            distance_error = math.hypot(dx, dy)
            
            #Desired heading to face the target
            desired_yaw = math.atan2(dy, dx)
            angular_error = self.normalize_angle(desired_yaw - current_yaw)
            
            #Compute dt safely (fallback to controller period if prev_time missing)
            current_time = self.get_clock().now()
            dt = 0.05 if self.prev_time is None else max(
                (current_time - self.prev_time).nanoseconds / 1e9, 1e-6)
            self.prev_time = current_time
            
            #Linear PID terms
            linear_derivative = (distance_error - self.prev_linear_error) / dt
            self.linear_error_sum += distance_error * dt
            linear_vel = (self.kp_linear * distance_error + 
                         self.ki_linear * self.linear_error_sum +
                         self.kd_linear * linear_derivative)
            
            #Angular PID terms
            angular_derivative = (angular_error - self.prev_angular_error) / dt
            self.angular_error_sum += angular_error * dt
            angular_vel = (self.kp_angular * angular_error +    
                          self.ki_angular * self.angular_error_sum +
                          self.kd_angular * angular_derivative)
            
            #Saturate velocities to configured maximums for safety
            linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
            
            #If heading error is large, slow down linear motion to prioritize rotation
            #Threshold here (0.5 rad ~ 28.6 degrees) can be tuned
            if abs(angular_error) > 0.5:
                linear_vel *= 0.5
            
            #Save errors for next derivative calculation
            self.prev_linear_error = distance_error
            self.prev_angular_error = angular_error
            
            return linear_vel, angular_vel
        except Exception as e:
            self.get_logger().error(f"Error in compute_control_commands: {e}\n{traceback.format_exc()}")
            return 0.0, 0.0

    def normalize_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        try:
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle
        except Exception as e:
            self.get_logger().error(f"Error in normalize_angle: {e}")
            return 0.0

    def is_trajectory_complete(self):
        """
        Check if the robot has reached the final waypoint within position tolerance.
        This function only checks distance to final pose once we've progressed to the end.
        """
        try:
            if not self.trajectory or not self.current_pose or len(self.trajectory.poses) == 0:
                return False
            
            # f we're already at or beyond the last index, check proximity to final pose
            if self.current_target_idx >= len(self.trajectory.poses) - 1:
                final_pose = self.trajectory.poses[-1]
                dx = final_pose.pose.position.x - self.current_pose.position.x
                dy = final_pose.pose.position.y - self.current_pose.position.y
                distance = math.hypot(dx, dy)
                return distance < self.pos_tolerance
            return False
        except Exception as e:
            self.get_logger().error(f"Error in is_trajectory_complete: {e}")
            return False
    

    def control_callback(self):
        """
        Main controller loop called periodically by self.control_timer.
        - If trajectory finished or missing data -> send zero velocities.
        - Otherwise compute a target via lookahead and publish computed cmd_vel.
        """
        try:
            #If controller is already finished or no trajectory/pose available -> stop robot
            if self.trajectory_complete or not self.trajectory or not self.current_pose:
                self.cmd_vel_pub.publish(Twist())
                return
            
            #Check completion condition
            if self.is_trajectory_complete():
                self.trajectory_complete = True
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Trajectory completed!")
                return
            
            #Choose a target point ahead on the trajectory
            target_pose, target_idx = self.find_target_point()
            if target_pose is None:
                #No valid target, stop for safety
                self.cmd_vel_pub.publish(Twist())
                return
            
            #Compute linear and angular velocities to reach the target
            linear_vel, angular_vel = self.compute_control_commands(target_pose)
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            #Occasional debug log to monitor controller progress without flooding logs
            if self.current_target_idx % 10 == 0:
                self.get_logger().debug(
                    f"Tracking point {target_idx}/{len(self.trajectory.poses)-1}, "
                    f"cmd_vel: linear={linear_vel:.3f}, angular={angular_vel:.3f}"
                )
        except Exception as e:
            #Any unexpected error should be logged but not crash the node
            self.get_logger().error(f"Error in control_callback: {e}\n{traceback.format_exc()}")


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
