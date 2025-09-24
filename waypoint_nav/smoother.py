#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import CubicSpline
from builtin_interfaces.msg import Time

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')
        
        #Publisher for the smoothed trajectory
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)

        #Example hardcoded waypoints for testing (x, y)
        self.waypoints = np.array([
            [0.0, 0.0],   [0.3, 0.6],   [0.7, 1.1],   [1.1, 1.7],   [1.5, 2.0],
            [2.0, 1.6],   [2.4, 1.0],   [2.7, 0.2],   [3.1, -0.5],  [3.6, -1.0],
            [4.0, -1.2],  [4.5, -1.3],  [5.0, -1.0],  [5.3, -0.5],  [5.7, 0.0],
            [6.0, 0.4],   [6.3, 0.8],   [6.6, 1.6],   [7.0, 1.3],   [7.5, 0.8],
            [8.0, 0.4],   [8.5, 0.0],   [8.9, -0.8],  [9.2, -1.4],  [9.6, -2.0],
            [9.8, -2.5],  [10.2, -1.7], [10.5, -1.0], [10.7, -0.2], [11.0, 0.7],
            [11.5, 1.2],  [12.0, 1.4],  [12.5, 1.1],  [12.8, 0.6],  [13.1, 0.0],
            [13.6, -0.5], [14.0, -1.1], [14.3, -1.9], [14.7, -2.2], [15.1, -1.9],
            [15.5, -1.3], [16.0, -0.6], [16.3, 0.1],  [16.6, 0.7],  [17.0, 1.4],
            [17.4, 1.7],  [17.8, 1.2],  [18.2, 0.4],  [18.6, -0.3], [19.0, -1.0]
        ])

        #Desired speed of the robot in m/s (used for time-parameterization)
        self.desired_velocity = 0.5  

        #Number of points to sample from the smoothed path
        self.num_samples = 250

        #Compute smoothed path + assign timing to each point
        self.smoothed_path, self.relative_times = self._compute_smoothed_path_with_time(
            self.waypoints, num_samples=self.num_samples
        )
        self.get_logger().info(f"Prepared smoothed path with {len(self.smoothed_path.poses)} points.")

        #Timer to periodically publish the trajectory (every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)  

    def _compute_smoothed_path_with_time(self, waypoints, num_samples=50, frame_id='odom'):
        """
        Take in waypoints → Fit cubic spline → Sample smooth path → Compute time stamps.
        Returns: Path msg + list of relative times for each point
        """
        if waypoints is None or waypoints.size == 0:
            self.get_logger().warn("No waypoints provided.")
            return Path(), []

        # Handle single waypoint case
        if len(waypoints) == 1:
            self.get_logger().warn("Only one waypoint provided.")
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(waypoints[0, 0])
            pose.pose.position.y = float(waypoints[0, 1])
            pose.pose.orientation.w = 1.0
            path_msg = Path()
            path_msg.header.frame_id = frame_id
            path_msg.poses.append(pose)
            return path_msg, [0.0]

        #Safety check for invalid velocity
        if self.desired_velocity <= 0.0:
            self.get_logger().warn("Desired velocity <= 0. Using default 0.5 m/s")
            self.desired_velocity = 0.5

        #Parametrize waypoints with an artificial 'time' index
        t = np.arange(len(waypoints))

        #Fit cubic splines separately for x and y
        cs_x = CubicSpline(t, waypoints[:, 0])
        cs_y = CubicSpline(t, waypoints[:, 1])

        #Sample evenly between first and last waypoint
        t_new = np.linspace(0, len(waypoints)-1, num_samples)
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        #Compute arc length of path → used to calculate traversal time
        diffs = np.diff(np.stack([x_smooth, y_smooth], axis=1), axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        arc_lengths = np.hstack(([0.0], np.cumsum(segment_lengths)))
        relative_times = arc_lengths / self.desired_velocity

        #Path message
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        for x, y in zip(x_smooth, y_smooth):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)

        return path_msg, relative_times

    def timer_callback(self):
        """Called periodically to publish the smoothed trajectory."""
        if not self.smoothed_path.poses:
            self.get_logger().warn("Smoothed path is empty. Nothing to publish.")
            return

        #Update path timestamp (important for consumers like controllers)
        now = self.get_clock().now()
        self.smoothed_path.header.stamp = now.to_msg()

        #Update each pose with a time offset (so controller knows "when" to be there)
        for idx, pose in enumerate(self.smoothed_path.poses):
            t_offset = int(self.relative_times[idx])
            nsec_offset = int((self.relative_times[idx] - t_offset) * 1e9)
            pose.header.stamp.sec = now.seconds_nanoseconds()[0] + t_offset
            pose.header.stamp.nanosec = now.seconds_nanoseconds()[1] + nsec_offset

        #Publish the path to /trajectory
        self.path_pub.publish(self.smoothed_path)
        self.get_logger().debug("Published /trajectory")

def main(args=None):
    #Standard ROS2 node init + spin loop
    rclpy.init(args=args)
    smoother_node = PathSmoother()
    try:
        rclpy.spin(smoother_node)
    except KeyboardInterrupt:
        pass
    finally:
        smoother_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
