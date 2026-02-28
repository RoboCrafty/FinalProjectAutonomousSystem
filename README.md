# Autonomous System Project 2026 - Cave Exploration Drone

This workspace contains the ROS 2 (Jazzy) packages necessary to simulate and control a quadrotor drone for an autonomous cave exploration mission. The architecture is modular, separating high-level mission logic from real-time trajectory generation and low-level control.

---

## 🏗️ Architecture Overview

The system operates as a continuous reactive pipeline:
**Decision Making ➡️ Trajectory Generator ➡️ Controller ➡️ Simulation**
*(with Odometry providing the feedback loop for state transitions and trajectory anchoring)*.



---

## 1. Decision Making Package (`decision_making_pkg`)

### Overview
The `decision_making_pkg` serves as the high-level "Brain" of the drone. It implements a **Finite State Machine (FSM)** in `state_machine.cpp` that sequences the mission phases.

### Implementation Details
* **Phase-Based Logic**: Uses an `enum` to transition between climbing, transiting, and completing the mission.
* **One-Shot Dispatch**: Utilizes a `target_sent_` flag to publish a goal to the navigation nodes exactly once per state transition, preventing trajectory resets.
* **Adaptive Target Helper**: Features a `sendTarget()` function with default parameters, allowing it to send either simple 3D coordinates or complex 3D orientations (Quaternions).
* **Proximity Detection**: Monitors `/current_state_est` and uses a 1.5m Euclidean distance threshold to determine when a target is reached.

### Mission States
| State | Action | Transition Condition |
| :--- | :--- | :--- |
| `WAYPOINT_HOVER` | Climb to $Z=20.0$ with a specific rotation command. | Distance $< 1.5m$ |
| `APPROACH_CAVE` | Transit to Cave Entrance ($X:-315.14, Y:8.38, Z:15.0$). | Distance $< 1.5m$ |
| `MISSION_COMPLETED` | Hold position and finalize mission flow. | End of Sequence |

---

## 2. Trajectory Generator Package (`trajectory_generator_pkg`)

### Overview
The `trajectory_generator_pkg` is the "Math Engine" that converts discrete waypoints into physically feasible 4D polynomial curves.

### Implementation Details
* **Polynomial Optimization**: Utilizes the `mav_trajectory_generation` library with a **Linear Solver** to ensure numerical stability and prevent matrix errors.
* **Adaptive Orientation**: Intelligently handles rotation. If a goal message contains "all-zero" orientation, the node maintains the drone's **current orientation**. If valid Quaternion data is provided, it incorporates the turn into the path.
* **Reactive Anchoring**: Every new trajectory is anchored to the drone's **true current position and velocity**, eliminating "jumps" during mid-air re-planning.
* **100Hz Setpoint Stream**: Samples the optimized trajectory every 10ms to provide high-frequency commands to the geometric controller.

### Topic Interfaces
* **Subscribes to**: `/current_state_est` (Odometry) and `/next_setpoint` (Goal).
* **Publishes to**: `/command/trajectory` (Multi-DOF Setpoints).

---

## 3. Lower-Level Packages (*Provided*)

### `controller_pkg`
Reads trajectory commands and calculates rotor speeds to maintain the desired state.
* **Subscribes to**: `/command/trajectory`, `/current_state_est`.

### `simulation`
The bridge between ROS 2 and the Unity world, simulating physics, collisions, and camera output.
* **Publishes to**: `/current_state_est`, `/realsense/depth/image`, `/realsense/rgb/image_raw`.

---

## 🚀 Usage / Testing

### 1. Build the Workspace
```bash
cd ~/Desktop/FinalProjectAutonomousSystem/ros2_ws
colcon build --packages-select decision_making_pkg trajectory_generator_pkg
source install/setup.bash