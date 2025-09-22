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

        self.waypoints = np.array([ 
            [0.0, 0.0], 
            [1.0, 0.5], 
            [2.0, 1.0], 
            [3.0, 1.0], 
            [4.0, 0.5], 
            [5.0, 0.0]
        ])

       
        """self.waypoints = np.array([
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
        ])"""

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
            [4.0, -2.0],   # Hard right
            [2.0, -2.5],   # Loop back left
            [0.0, -2.0],   # Full U-turn path
            [1.5, -0.5],   # Quick slalom cut
            [3.0, -1.5],
            [4.5, -0.5],
        ]) """




        self.path_msg = self._compute_smoothed_path(self.waypoints, num_samples=100)
        self.get_logger().info(f"Prepared smoothed path with {len(self.path_msg.poses)} points")

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def _compute_smoothed_path(self, waypoints, num_samples=50, frame_id='odom'):

        t = np.arange(len(waypoints)) 
        cs_x = CubicSpline(t, waypoints[:, 0])
        cs_y = CubicSpline(t, waypoints[:, 1])

        t_new = np.linspace(0, len(waypoints)-1, num_samples)
        x_smooth = cs_x(t_new)
        y_smooth = cs_y(t_new)

        path_msg = Path()
        path_msg.header.frame_id = frame_id

        for x, y in zip(x_smooth, y_smooth):
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

        return path_msg

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        for pose in self.path_msg.poses:
            pose.header.stamp = now
        self.path_pub.publish(self.path_msg)
        self.get_logger().debug("Published /trajectory")

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
