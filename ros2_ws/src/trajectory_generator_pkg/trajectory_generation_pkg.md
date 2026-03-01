# Reactive Trajectory Generator Package

## Overview
The `trajectory_generator_pkg` provides a **reactive, high-frequency trajectory generator** designed for autonomous quadrotor navigation. Unlike static path planners, this node specializes in real-time motion generation by anchoring the start of every new trajectory to the drone's **actual current position and velocity**. 

By utilizing the `mav_trajectory_generation` library and a **Linear Optimization** solver, it ensures that transitions between setpoints are smooth, continuous, and physically feasible, effectively eliminating the "jumping" behavior caused by starting paths from stale or incorrect coordinates.



## Key Functionalities
* **Dynamic Re-planning**: Automatically computes a new minimum-acceleration polynomial segment as soon as a target is received on the `/next_setpoint` topic.
* **Adaptive Orientation**: Intelligently handles rotation commands. If a valid orientation is provided in the goal message, the drone will rotate to that pose; otherwise, it automatically maintains its **current orientation** during the flight.
* **Real-time Synchronization**: Pulls the latest state (position, velocity, and orientation) from `/current_state_est` to ensure the trajectory start matches the drone's physical reality.
* **High-Frequency Setpoint Stream**: Samples the generated trajectory at **100Hz** (10ms) to provide a dense stream of setpoints for the low-level geometric controller.
* **Kinematic Constraint Enforcement**: Limits the generated path based on configurable velocity (`max_v`) and acceleration (`max_a`) parameters.

## Topic Interfaces

### Subscribed Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/current_state_est` | `nav_msgs/msg/Odometry` | Receives the real-time position, velocity, and orientation of the drone. |
| `/next_setpoint` | `geometry_msgs/msg/PoseStamped` | Receives the target goal coordinates and optional orientation. Triggers a new trajectory calculation. |

### Published Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | Publishes the sampled $x, v, a$ points and orientation for the low-level controller. |

## Parameters
* **`max_v`** (Default: `15.0`): The maximum target velocity in meters per second.
* **`max_a`** (Default: `5.0`): The maximum target acceleration in meters per second squared.



## Quick Start

### 1. Move to a Position (Maintain Current Facing)
To send the drone to a specific location while keeping its current orientation, publish a message with an uninitialized (all zero) orientation:
```bash
ros2 topic pub --once /next_setpoint geometry_msgs/msg/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: -36.0, y: 10.0, z: 15.0}}}"