import rclpy
import pytest
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path

from waypoint_nav.smoother import PathSmoother
from waypoint_nav.controller import TrajectoryController

@pytest.fixture(scope="module")
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()

@pytest.fixture
def path_smoother_node(rclpy_init_shutdown):
    node = PathSmoother()
    yield node
    node.destroy_node()

@pytest.fixture
def trajectory_controller_node(rclpy_init_shutdown):
    node = TrajectoryController()
    yield node
    node.destroy_node()

# PathSmoother Tests
def test_path_smoother_generated_path(path_smoother_node):
    #Test that smoothed path has correct number of points
    smoothed_path = path_smoother_node.smoothed_path
    assert len(smoothed_path.poses) == path_smoother_node.num_samples

def test_path_smoother_relative_times(path_smoother_node):
    #Test that relative times are positive and monotonically increasing
    times = path_smoother_node.relative_times
    assert all(t >= 0 for t in times)
    assert np.all(np.diff(times) >= 0)

def test_path_smoother_empty_waypoints():
    #Test empty waypoints are handled gracefully
    node = PathSmoother()
    path, times = node._compute_smoothed_path_with_time(np.array([]))
    assert isinstance(path, Path)
    assert len(path.poses) == 0
    assert times == []

#TrajectoryController Tests
def test_controller_receive_trajectory(trajectory_controller_node):
    #Create simple trajectory
    path = Path()
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = i * 0.5
        pose.pose.position.y = 0.0
        path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)
    assert trajectory_controller_node.trajectory is not None
    assert len(trajectory_controller_node.trajectory.poses) == 5

def test_find_target_point(trajectory_controller_node):
    #Simple straight-line trajectory
    path = Path()
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = i * 0.5
        pose.pose.position.y = 0.0
        path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)
    
    # Simulate robot near start
    class DummyPose:
        position = type('obj', (object,), {'x':0.0, 'y':0.0})
        orientation = type('obj', (object,), {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()
    
    target_pose, idx = trajectory_controller_node.find_target_point()
    assert target_pose is not None
    assert idx >= 0

def test_compute_control_commands(trajectory_controller_node):
    #Test PID control outputs reasonable velocities
    class DummyPose:
        position = type('obj', (object,), {'x':0.0, 'y':0.0})
        orientation = type('obj', (object,), {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()
    
    target_pose = PoseStamped()
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 0.0
    
    linear_vel, angular_vel = trajectory_controller_node.compute_control_commands(target_pose)
    assert isinstance(linear_vel, float)
    assert isinstance(angular_vel, float)

def test_trajectory_completion(trajectory_controller_node):
    #Test trajectory completion detection
    path = Path()
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)
    
    class DummyPose:
        position = type('obj', (object,), {'x':0.0, 'y':0.0})
        orientation = type('obj', (object,), {'x':0.0, 'y':0.0, 'z':0.0, 'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()
    
    assert trajectory_controller_node.is_trajectory_complete() == True

