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

        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        """self.waypoints = np.array([ 
            [0.0, 0.0], 
            [1.0, 0.5], 
            [2.0, 1.0], 
            [3.0, 1.0], 
            [4.0, 0.5], 
            [5.0, 0.0]
        ]) """


        """
        self.waypoints = np.array([
            [0.0, 0.0],   
            [1.0, 1.0],    
            [2.5, -0.5],
            [3.0, 2.0],    
            [4.5, 1.5],    
            [5.0, 3.0],   
            [6.5, 0.0],   
            [7.0, -1.5],   
            [8.0, 1.0],    
            [9.0, 0.0],    
        ]) """


        """self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 1.0],
            [2.0, -1.0],
            [3.0, 1.0],
            [4.0, -1.0],
            [5.0, 1.0],
            [6.0, 0.0],
        ]) """


        """self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [2.0, 1.0],
            [2.0, 2.5],
            [1.0, 3.5],
            [0.0, 3.0],
            [-0.5, 2.0],
            [0.0, 1.0],
            [0.5, 0.5],
        ]) """


        """ self.waypoints = np.array([
            [0.0, 0.0],
            [2.0, 0.0],
            [4.0, 0.0],
            [4.0, -2.0],   
            [2.0, -2.5],   
            [0.0, -2.0],  
            [1.5, -0.5],   
            [3.0, -1.5],
            [4.5, -0.5],
        ]) """

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 0.3],
            [2.0, 0.8],
            [3.0, 0.2],
            [4.0, -0.5],
            [5.0, -1.0],
            [6.0, -0.7],
            [7.0, 0.1],
            [8.0, 0.9],
            [9.0, 1.5],
        ]) """

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [1.5, -0.5],
            [2.0, -1.5],
            [2.5, -0.7],
            [3.5, 0.0],
            [4.0, 1.0],
            [5.0, 1.5],
            [6.0, 1.0],
            [7.5, 0.5],
        ]) """

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [2.0, 2.5],
            [3.0, 1.5],
            [3.5, 0.5],
            [2.5, -0.5],
            [1.5, -1.5],
            [0.0, -1.0],
        ]) """

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [0.5, -0.4],
            [1.0, -0.9],
            [1.5, -1.1],
            [2.0, -1.0],
            [2.5, -0.6],
            [3.0, -0.2],
            [3.5, 0.3],
            [4.0, 0.8],
            [4.5, 1.1],
            [5.0, 1.3],
            [5.5, 1.2],
            [6.0, 1.0],
            [6.5, 0.5],
            [7.0, 0.0],
        ]) """

        self.waypoints = np.array([
            [0.0, 0.0],
            [0.7, 0.3],
            [1.4, 0.8],
            [2.1, 1.3],
            [2.8, 1.1],
            [3.5, 0.7],
            [4.0, 0.0],
            [4.3, -0.6],
            [4.5, -1.2],
            [4.8, -1.8],
            [5.1, -1.5],
            [5.7, -1.0],
            [6.1, -0.3],
            [6.5, 0.5],
            [7.0, 1.2],
            [7.5, 1.8],
            [8.0, 1.5],
            [8.5, 0.7],
            [9.0, 0.0],
            [9.5, -0.5]
        ])

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [0.5, -0.2],
            [1.5, -0.4],
            [2.5, -0.1],
            [3.0, 0.5],
            [3.5, 1.0],
            [4.0, 1.7],
            [5.0, 2.0],
            [6.0, 1.5],
            [7.0, 0.8],
        ]) """

        """ self.waypoints = np.array([
            [0.0, 0.0],
            [1.0, 0.0],
            [2.0, 0.5],
            [2.5, 1.5],
            [3.0, 2.5],
            [4.0, 2.0],
            [5.0, 1.0],
            [6.0, 0.5],
            [7.0, 0.0],
            [8.0, -0.5],
        ]) """

        self.desired_velocity = 0.5  
        self.num_samples = 100

        self.smoothed_path, self.relative_times = self._compute_smoothed_path_with_time(
            self.waypoints,
            num_samples=self.num_samples
        )
        self.get_logger().info(f"prepared smoothed path with {len(self.smoothed_path.poses)} points and time-parameterization")

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def _compute_smoothed_path_with_time(self, waypoints, num_samples=50, frame_id='odom'):
        t = np.arange(len(waypoints)) 
        cs_x = CubicSpline(t, waypoints[:, 0])
        cs_y = CubicSpline(t, waypoints[:, 1])

        t_new = np.linspace(0, len(waypoints)-1, num_samples)
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        diffs = np.diff(np.stack([x_smooth, y_smooth], axis=1), axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        arc_lengths = np.hstack(([0.0], np.cumsum(segment_lengths)))  

        if self.desired_velocity <= 0.0:
            self.desired_velocity = 0.5 
        relative_times = arc_lengths / self.desired_velocity  

        path_msg = Path()
        path_msg.header.frame_id = frame_id

        for idx, (x, y) in enumerate(zip(x_smooth, y_smooth)):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg, relative_times

    def timer_callback(self):
        now = self.get_clock().now()
        self.smoothed_path.header.stamp = now.to_msg()
        for idx, pose in enumerate(self.smoothed_path.poses):
            t_offset = int(self.relative_times[idx])
            nsec_offset = int((self.relative_times[idx] - t_offset) * 1e9)
            pose.header.stamp.sec = now.seconds_nanoseconds()[0] + t_offset
            pose.header.stamp.nanosec = now.seconds_nanoseconds()[1] + nsec_offset
        self.path_pub.publish(self.smoothed_path)
        self.get_logger().debug("Published time-parameterized /trajectory")

def main(args=None):
    rclpy.init(args=args)
    node = PathSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
