# Path Smoothing and Trajectory Tracking for Differential Drive Robot

## Overview

This repository implements path smoothing, time-parameterized trajectory generation, and trajectory tracking for a differential drive robot navigating through 2D waypoints using ROS2 as part of the assessment for 10xConstruction.ai's Robotics Software Apprentice position.

---

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Setup Instructions](#setup-instructions)
- [Implementation Details](#implementation-details)
  - [Path Smoothing](#path-smoothing)
  - [Trajectory Generation](#trajectory-generation)
  - [Tracking Controller](#tracking-controller)
- [Simulation and Results](#simulation-and-results)
- [Extending to Real Robot](#extending-to-real-robot)
- [AI Tool Support](#ai-tool-support)
- [Testing and Error Handling](#testing-and-error-handling)

---

## Introduction
Developed a package consisting of two modular ROS2 nodes:

### PathSmoother
Generates a smooth, continuous, and time-parameterized trajectory from discrete waypoints using cubic spline interpolation.
### TrajectoryController
Implements a PID-based pure pursuit trajectory tracking controller that takes the smoothed trajectory as input and outputs velocity commands for the robot.

## Features

### Path Smoothing
- Converts discrete 2D waypoints into a smooth, continuous trajectory using cubic spline interpolation.
  
### Time Parameterization
- Assigns timestamps to trajectory points based on desired velocity
  
### Outputs a time-parameterized trajectory (nav_msgs/Path)
- Implements a PID-based pure pursuit controller for both linear and angular velocity and publishes standard ROS2 velocity commands (geometry_msgs/Twist) for differential drive robots.

### ROS2 Integration
- Works with any differential drive robot supporting /cmd_vel and /odom topics (tested in simulation with Turtlebot3).

### Logging & Debugging Support
- Informational logs on trajectory reception and completion.

## Setup Instructions

### 0. Prerequisites
- ROS2 Humble (or later) installed on your system.
- A working ROS2 workspace with colcon build tools.

### 1. Clone the repository

```bash
cd your_ws/src
git clone git@github.com:ShauryaJain03/.git

```
### 2. Build the Package
```bash
cd ..
colcon build
source install/setup.bash
```

### 3. Run the Nodes using the provided launch file
```bash
ros2 launch smooth_waypoint_navigation smooth_waypoint.launch.py
```
---

## Implementation Details

### Path Smoothing
- Implemented in the **`PathSmoother`** node.  
- Uses **cubic spline interpolation** to transform discrete waypoints into a continuous, smooth path.  
- The path is sampled at a configurable number of points (`num_samples`), ensuring sufficient resolution for control.  
- Each sampled point is represented as a `geometry_msgs/PoseStamped`, aggregated into a `nav_msgs/Path` message.  
- **Advantages of cubic splines**:  
  - Smooth position and first derivative (continuous velocity profile).  
  - Avoids sharp turns or discontinuities present in raw waypoints.  

### Trajectory Generation
- The smoothed path is **time-parameterized** based on a desired constant velocity. 
- This produces a **trajectory with relative timing**, ensuring the robot executes the path at the expected speed.  
- Published as a `nav_msgs/Path` where each pose includes a timestamp in its header.  
- Ready for consumption by the controller without further preprocessing.  

### Tracking Controller
- Implemented in the **`TrajectoryController`** node.  
- **Core components**:  
  - **Target Point Selection** → Uses a **lookahead distance** to choose a point ahead of the robot for stable tracking (Pure Pursuit).  
  - **PID Control**:  
    - Linear velocity = PID(distance error)  
    - Angular velocity = PID(angular error)  
  - Velocity outputs are clamped to configurable max values.  
  - Linear velocity is reduced during large heading errors to prioritize orientation alignment.  
- **Safety & completion handling**:  
  - Robot stops once the final trajectory point is reached within a position tolerance.  
  - Zero velocity is published if no trajectory or odometry data is available.  
- Publishes velocity commands to `/cmd_vel`, making it compatible with any differential-drive robot.  

---

## Simulation and Results


---

## Extending to Real Robot

### Key Steps for Real-World Deployment
1. **Hardware Interface**  
   - Ensure the robot can consume velocity commands on `/cmd_vel` and publish odometry on `/odom`.  
   - Most ROS2 supported robots (TurtleBot versions, Jackal, etc.) already provide these interfaces.  

2. **Localization**  
   - In simulation, perfect ground-truth odometry is available.  
   - On a real robot, use sensor fusion (e.g., **robot_localization** with wheel odometry, IMU) to provide a reliable `/odom` estimate.  

3. **Coordinate Frames**  
   - Verify TF transforms between `odom`, `base_link`.  
   - Use **tf2_ros** or robot’s built-in localization stack to maintain consistency.  

4. **Controller Tuning**  
   - PID gains and lookahead distance may need re-tuning to account for:  
     - Robot wheelbase and dynamics.  
     - Sensor noise and latency.  
     - Slippage or uneven terrain.  

5. **Safety & Fail-safes**  
   - Integrate obstacle detection (e.g., LiDAR + costmaps) before deployment in cluttered environments.  
   - Implement emergency stop (E-Stop) functionality to override `/cmd_vel`.  

### Example Real-World Workflow
1. Upload waypoints → generate smoothed trajectory using `PathSmoother`.  
2. Launch `TrajectoryController` → robot tracks the path in real-time using odometry.  

By following these steps, the same ROS2 nodes can be used on real robots with minimal changes, ensuring smooth, reliable, and safe trajectory execution.


---

## AI Tool Support

- Perplexity - Researching different implementations for solving the given problem
- ChatGPT - Used for generating readme and filling content along with test cases of different waypoints, debugging.  

---

## Testing and Error Handling

---
