#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.trajectory_sub = self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.modified_path_pub = self.create_publisher(Path, '/modified_trajectory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        self.original_trajectory = None
        self.current_pose = None
        self.scan_data = None
        self.visited_waypoints = set()

        self.safety_radius = 0.3      
        self.detour_distance = 0.4     
        self.lookahead_points = 10    
        self.waypoint_tolerance = 0.1  

        self.timer = self.create_timer(0.1, self.avoidance_loop)

        self.get_logger().info("Obstacle Avoidance Node Initialized")

    def trajectory_callback(self, msg):
        self.original_trajectory = msg
        self.visited_waypoints.clear() 

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.scan_data = msg

    def avoidance_loop(self):
        if self.original_trajectory is None or self.current_pose is None:
            return

        modified_path = Path()
        modified_path.header = self.original_trajectory.header
        modified_path.poses = list(self.original_trajectory.poses)

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        self.update_visited_waypoints(modified_path, robot_x, robot_y)

        if self.scan_data is not None:
            closest_idx = self.find_closest_waypoint(modified_path, robot_x, robot_y)
            for i in range(closest_idx, min(closest_idx + self.lookahead_points, len(modified_path.poses))):
                wp = modified_path.poses[i]
                if self.is_waypoint_blocked(wp):
                    self.get_logger().info(f"Obstacle detected near waypoint {i}, generating detour")
                    detour_path = self.generate_detour(self.current_pose, wp, modified_path.poses, i)
                    if detour_path:
                        modified_path.poses = modified_path.poses[:closest_idx] + detour_path + modified_path.poses[i+1:]
                    break

        self.modified_path_pub.publish(modified_path)

        self.publish_markers(modified_path)

    def update_visited_waypoints(self, path, x, y):
        for i, pose in enumerate(path.poses):
            if i in self.visited_waypoints:
                continue
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            if math.hypot(dx, dy) < self.waypoint_tolerance:
                self.visited_waypoints.add(i)

    def find_closest_waypoint(self, path, x, y):
        min_dist = float('inf')
        idx = None
        for i, pose in enumerate(path.poses):
            if i in self.visited_waypoints:
                continue
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                idx = i
        return idx if idx is not None else 0

    def is_waypoint_blocked(self, waypoint):
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        for i, r in enumerate(self.scan_data.ranges):
            if np.isinf(r) or np.isnan(r):
                continue
            angle = angle_min + i * angle_increment
            scan_x = r * math.cos(angle)
            scan_y = r * math.sin(angle)
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            robot_yaw = self.yaw_from_quaternion(self.current_pose.orientation)
            global_x = robot_x + scan_x * math.cos(robot_yaw) - scan_y * math.sin(robot_yaw)
            global_y = robot_y + scan_x * math.sin(robot_yaw) + scan_y * math.cos(robot_yaw)
            dx = waypoint.pose.position.x - global_x
            dy = waypoint.pose.position.y - global_y
            if math.hypot(dx, dy) < self.safety_radius:
                return True
        return False

    def generate_detour(self, robot_pose, blocked_wp, trajectory_poses, blocked_idx):
        rx = robot_pose.position.x
        ry = robot_pose.position.y
        bx = blocked_wp.pose.position.x
        by = blocked_wp.pose.position.y
        vec_x = bx - rx
        vec_y = by - ry
        norm = math.hypot(vec_x, vec_y)
        if norm == 0:
            return None
        vec_x /= norm
        vec_y /= norm

        detour_pose = PoseStamped()
        detour_pose.pose.position.x = rx - vec_y * self.detour_distance
        detour_pose.pose.position.y = ry + vec_x * self.detour_distance
        detour_pose.pose.orientation.w = 1.0
        if not self.is_point_blocked(detour_pose):
            next_idx = min(blocked_idx + 1, len(trajectory_poses)-1)
            return [detour_pose, trajectory_poses[next_idx]]

        detour_pose.pose.position.x = rx + vec_y * self.detour_distance
        detour_pose.pose.position.y = ry - vec_x * self.detour_distance
        if not self.is_point_blocked(detour_pose):
            next_idx = min(blocked_idx + 1, len(trajectory_poses)-1)
            return [detour_pose, trajectory_poses[next_idx]]

        return None

    def is_point_blocked(self, pose):
        angle_min = self.scan_data.angle_min
        angle_increment = self.scan_data.angle_increment
        for i, r in enumerate(self.scan_data.ranges):
            if np.isinf(r) or np.isnan(r):
                continue
            angle = angle_min + i * angle_increment
            scan_x = r * math.cos(angle)
            scan_y = r * math.sin(angle)
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            robot_yaw = self.yaw_from_quaternion(self.current_pose.orientation)
            global_x = robot_x + scan_x * math.cos(robot_yaw) - scan_y * math.sin(robot_yaw)
            global_y = robot_y + scan_x * math.sin(robot_yaw) + scan_y * math.cos(robot_yaw)
            dx = pose.pose.position.x - global_x
            dy = pose.pose.position.y - global_y
            if math.hypot(dx, dy) < self.safety_radius:
                return True
        return False

    def yaw_from_quaternion(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_markers(self, path):
        marker_array = MarkerArray()
        path_marker = Marker()
        path_marker.header.frame_id = path.header.frame_id
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        for pose in path.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.0
            path_marker.points.append(p)
        marker_array.markers.append(path_marker)

        if self.current_pose is not None:
            arrow = Marker()
            arrow.header.frame_id = path.header.frame_id
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "robot_heading"
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.3
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.r = 1.0
            arrow.color.g = 0.0
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            arrow.points = []
            p_start = Point()
            p_start.x = self.current_pose.position.x
            p_start.y = self.current_pose.position.y
            p_start.z = 0.0
            yaw = self.yaw_from_quaternion(self.current_pose.orientation)
            p_end = Point()
            p_end.x = p_start.x + 0.5 * math.cos(yaw)
            p_end.y = p_start.y + 0.5 * math.sin(yaw)
            p_end.z = 0.0
            arrow.points.append(p_start)
            arrow.points.append(p_end)
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
