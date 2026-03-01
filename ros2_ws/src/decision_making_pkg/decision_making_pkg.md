# Decision Making Package

## Overview
The `decision_making_pkg` acts as the high-level **Decision Making Layer** for the autonomous quadrotor system. It implements a robust **Finite State Machine (FSM)** in `state_machine.cpp` that coordinates drone behavior across mission phases.

It monitors drone progress via odometry and dispatches goal targets to the **Reactive Trajectory Generator** once mission criteria are met.

## Core Functionalities
* **Sequential State Management**: Ensures the drone completes tasks in a logical order (e.g., climbing to safety before transit).
* **One-Shot Dispatch Logic**: Uses a `target_sent_` flag to publish exactly once per state transition, ensuring stable flight.
* **Dynamic Arrival Checking**: Calculates Euclidean distance to target with a 1.5m tolerance threshold to trigger phase changes.

## Mission Logic
| State | Action | Transition Condition |
| :--- | :--- | :--- |
| `WAYPOINT_HOVER` | Climb to $Z=20.0$ with rotation. | Distance $< 1.5m$ |
| `APPROACH_CAVE` | Transit to Cave Entrance. | Distance $< 1.5m$ |
| `MISSION_COMPLETED` | Hold position. | End of Sequence |

## Topic Interfaces
* **/current_state_est** (`nav_msgs/msg/Odometry`): Tracks drone position and orientation.
* **/next_setpoint** (`geometry_msgs/msg/PoseStamped`): Dispatches high-level goal coordinates.

## Usage
```bash
ros2 run decision_making_pkg state_machine_node