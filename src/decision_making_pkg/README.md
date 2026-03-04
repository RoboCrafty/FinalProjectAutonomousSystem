# Decision Making Package

## Overview

The `decision_making_pkg` serves as the high-level **Decision Making Layer** for the autonomous quadrotor. It implements a **Finite State Machine (FSM)** in `state_machine.cpp` that sequences mission phases, from initial takeoff to frontier exploration and final mission completion.

It processes odometry data to track progress and dispatches goal targets to the trajectory generator. It also manages the lifecycle of the mapping and exploration nodes.

## Core Functionalities

* **Sequential Mission Control**: Manages transitions between high-level states: `APPROACH_CAVE`, `EXPLORE_CAVE`, and `MISSION_COMPLETED`.
* **Sub-Step Sequencing**: Uses internal sub-steps within states to handle complex maneuvers, such as the multi-point cave entry sequence.
* **One-Shot Dispatch**: A `target_sent_` flag ensures that goal coordinates are published only once per transition to prevent flickering in the trajectory generator.
* **Environmental Reset**: Automatically triggers an Octomap reset via service call upon reaching the cave entrance to ensure a clean map for exploration.

## Mission Logic Flow

| State | Sub-Step / Action | Transition Condition |
| --- | --- | --- |
| **APPROACH_CAVE** | 0: Climb to Hover point ($Z=25.0$). | Distance $< 1.5$m |
|  | 1: Transit to Cave Entrance. | Distance $< 1.5$m & Reset Octomap |
|  | 2: Enter Cave Interior. | Distance $< 1.5$m |
| **EXPLORE_CAVE** | Wait 4s for sensors, then enable Frontier Explorer. | Received `exploration_complete` |
| **MISSION_COMPLETED** | Disable Exploration and hover. | End of Sequence |

## Topic & Service Interfaces

### Subscriptions

* **/current_state_est** (`nav_msgs/msg/Odometry`): Provides real-time position for distance threshold checking.
* **/exploration_complete** (`std_msgs/msg/Bool`): Listens for a signal from the exploration node to transition to the final state.

### Publications

* **/next_setpoint** (`geometry_msgs/msg/PoseStamped`): Dispatches navigation goals to the reactive controller.
* **/enable_exploration** (`std_msgs/msg/Bool`): High-level toggle to start or stop the frontier exploration algorithm.

### Services

* **/octomap_server/reset** (`std_srvs/srv/Empty`): Called to clear the 3D occupancy map before entering the cave.

## Usage

To launch the decision-making node:

```bash
ros2 run decision_making_pkg state_machine_node

```