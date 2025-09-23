import rclpy
import pytest
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry

from waypoint_nav.smoother import PathSmoother
from waypoint_nav.controller import TrajectoryController

#Fixtures to initialize nodes
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

#smoother Tests
def test_path_smoother_generated_path(path_smoother_node):
    #Test normal smoothed path generation
    smoothed_path = path_smoother_node.smoothed_path
    assert len(smoothed_path.poses) == path_smoother_node.num_samples

def test_path_smoother_relative_times(path_smoother_node):
    #Relative times should be positive and increasing
    times = path_smoother_node.relative_times
    assert all(t >= 0 for t in times)
    assert np.all(np.diff(times) >= 0)

def test_path_smoother_empty_waypoints(path_smoother_node):
    #Test empty waypoints are handled gracefully
    path, times = path_smoother_node._compute_smoothed_path_with_time(np.array([]))
    assert isinstance(path, Path)
    assert len(path.poses) == 0
    assert times == []

def test_path_smoother_single_waypoint(path_smoother_node):
    #Test single waypoint handling
    wp = np.array([[1.0, 2.0]])
    path, times = path_smoother_node._compute_smoothed_path_with_time(wp)
    assert isinstance(path, Path)
    assert len(path.poses) == 1
    assert times == [0.0]

#Controller Tests
def test_controller_receive_trajectory(trajectory_controller_node):
    #test normal trajectory reception
    path = Path()
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = i * 0.5
        pose.pose.position.y = 0.0
        path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)
    assert trajectory_controller_node.trajectory is not None
    assert len(trajectory_controller_node.trajectory.poses) == 5

def test_controller_empty_trajectory(trajectory_controller_node):
    #Controller should handle empty trajectory
    path = Path()
    trajectory_controller_node.trajectory_callback(path)
    assert trajectory_controller_node.trajectory is None

def test_find_target_point(trajectory_controller_node):
    #Test target point selection logic
    path = Path()
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = i * 0.5
        pose.pose.position.y = 0.0
        path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)

    #simulate robot near start
    class DummyPose:
        position = type('obj', (object,), {'x':0.0, 'y':0.0})
        orientation = type('obj', (object,), {'x':0.0,'y':0.0,'z':0.0,'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()

    target_pose, idx = trajectory_controller_node.find_target_point()
    assert target_pose is not None
    assert idx >= 0

def test_compute_control_commands(trajectory_controller_node):
    #PID outputs reasonable float velocities
    class DummyPose:
        position = type('obj', (object,), {'x':0.0,'y':0.0})
        orientation = type('obj', (object,), {'x':0.0,'y':0.0,'z':0.0,'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()

    target_pose = PoseStamped()
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 0.0

    linear_vel, angular_vel = trajectory_controller_node.compute_control_commands(target_pose)
    assert isinstance(linear_vel, float)
    assert isinstance(angular_vel, float)

def test_missing_odometry(trajectory_controller_node):
    #Controller should handle missing odometry safely
    trajectory_controller_node.current_pose = None
    target_pose = PoseStamped()
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 0.0
    linear_vel, angular_vel = trajectory_controller_node.compute_control_commands(target_pose)
    assert linear_vel == 0.0
    assert angular_vel == 0.0

def test_trajectory_completion(trajectory_controller_node):
    #Test trajectory completion detection
    path = Path()
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    path.poses.append(pose)
    trajectory_controller_node.trajectory_callback(path)

    class DummyPose:
        position = type('obj', (object,), {'x':0.0,'y':0.0})
        orientation = type('obj', (object,), {'x':0.0,'y':0.0,'z':0.0,'w':1.0})
    trajectory_controller_node.current_pose = DummyPose()
    assert trajectory_controller_node.is_trajectory_complete() == True
