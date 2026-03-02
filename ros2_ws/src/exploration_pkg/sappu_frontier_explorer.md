# README.md

## Architecture Overview

The system operates as a continuous reactive pipeline:
Decision Making -> Frontier Explorer -> Trajectory Generator -> Controller -> Simulation.

* Decision Making (state_machine.cpp): Sequentially manages mission phases: climbing, approaching the cave entrance, and handing over control to the explorer. It includes a service call to reset the Octomap upon entering the cave to ensure a clean mapping environment.
* Frontier Explorer (sappu_frontier_explorer.cpp): The core autonomous engine. It analyzes the occupancy grid to determine movement and builds a persistent memory of the cave's topology.
* Trajectory Generator: Receives 3D setpoints and generates physically feasible 4D polynomial trajectories sampled at 100Hz.
* Controller: A geometric controller that calculates rotor speeds based on trajectory commands and current state estimation.

---

## Exploration Algorithm Logic

The sappu_frontier_explorer implements a Depth-First Search (DFS) approach on a topological graph to handle the unique challenges of sub-terrain environments, such as forks and loops.

### 1. Perception and Splitting

* 180 FOV: The drone only considers frontiers within a 180-degree horizontal field of view in front of its current heading.
* Candidate Generation: Detected frontiers are split into Left and Right groups based on their relative angle to the drone.
* Averaging: The algorithm computes the average 3D position (centroid) for both the left and right groups to find two potential candidate points.

### 2. Decision Rules

* Fork Detection: If both left and right candidates exist, the drone checks the midpoint between them.
* Wall Check: If the midpoint is blocked by a wall (occupied voxel in Octomap), the area is flagged as a True Fork.
* Always Left: In a True Fork, the drone always prioritizes the Left candidate.
* Wide Corridors: If the midpoint is free (not a wall), the drone proceeds toward the center to maintain a safe distance from walls.

### 3. Graph Memory and States

* Area Nodes: The drone creates an AreaNode every 7 meters or whenever a True Fork is detected.
* Exploring Mode: The default state where the drone pursues new frontiers using the Always Left rule.
* Loop Closure: If the drone comes within 4 meters of an existing node (that is not its immediate parent), it identifies a loop. It marks the current path as closed and switches to Backtracking Mode.
* Backtracking Mode: The drone follows the graph nodes in reverse (toward the parent_id). If it reaches a Fork Node where the right branch has not been taken, it switches back to Exploring Mode to map that branch.

---

## Installation and Usage

### Prerequisites

* Ubuntu 24.04 (or compatible Linux distribution)
* ROS 2 Jazzy
* Octomap and TF2 libraries

### Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select decision_making_pkg exploration_pkg trajectory_generator_pkg controller_pkg simulation
source install/setup.bash

```

### Running the Mission

1. Launch the Unity Simulation environment.
2. Launch the ROS bridge and simulation nodes:

```bash
ros2 launch simulation simulation.launch.py

```

3. The mission begins automatically with the state_machine_node.

---

## Project Requirements Fulfillment

* Autonomous Navigation: The system requires no manual waypoints inside the cave, relying entirely on the frontier exploration algorithm.
* Mapping: Generates a 3D voxel-grid representation of the cave using Octomap.
* Architecture: Modular design with clear separation between high-level logic and low-level control.