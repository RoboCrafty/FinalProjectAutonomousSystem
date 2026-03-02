This updated README reflects the transition to the master-slave architecture, the inclusion of the C++ Hunting node, the 3D NBV Explorer, and the optimized performance configurations for virtual machine environments.

# Autonomous System Project 2026 - Cave Exploration Drone

This workspace contains the ROS 2 (Jazzy) packages necessary to simulate and control a quadrotor drone for an autonomous cave exploration mission. The architecture is modular, separating high-level mission logic from real-time trajectory generation, 3D exploration, and visual pursuit.

---

## 🏗️ Architecture Overview

The system operates as a **Master-Slave Finite State Machine**:
**State Machine (Master) ↔️ {Explorer / Hunter} ➡️ Trajectory Generator ➡️ Controller**
*(All nodes are synchronized via a central `/enable` topic logic to prevent control conflicts)*.

---

## 1. Decision Making Package (`decision_making_pkg`)

### Overview

The `decision_making_pkg` serves as the high-level "Brain." It has been updated to manage multiple external nodes and track mission progress across multiple targets.

### Implementation Details

* **Multi-Node Coordination**: Manages lifecycle toggles for the 3D Explorer and Lantern Hunter via `/enable_exploration` and `/enable_hunting`.
* **Lantern Counter**: Tracks progress toward the 4-lantern goal, transitioning from hunting back to exploration until the count is reached.
* **Confidence Filtering**: Features a semantic area filter (Area > 200.0) to ignore visual noise and only trigger hunting on confirmed lanterns.
* **Octomap Integration**: Automatically triggers a `/octomap_server/reset` service call upon cave entry to clear memory and ensure fresh pathfinding.

### Mission States

| State | Action | Transition Condition |
| --- | --- | --- |
| `WAYPOINT_HOVER` | Initial climb to mission altitude. | Distance $< 1.5m$ |
| `APPROACH_CAVE` | Transit to cave entrance and reset Octomap. | Distance $< 1.5m$ |
| `EXPLORE_CAVE` | Activate 3D NBV Explorer node. | Lantern Spotted (Area $> 200$) |
| `HUNTING_LANTERN` | Activate C++ Hunting node for visual pursuit. | Signal `/hunting_done` received |
| `MISSION_COMPLETED` | Finalize mission after 4 lanterns are secured. | Counter $\geq 4$ |

---

## 2. Exploration Package (`exploration_pkg`)

### Overview

Implements a C++ **3D Next-Best-View (NBV) Explorer** that uses Octomap data to navigate unknown environments.

### Implementation Details

* **Octomap Dependency**: Subscribes to `/octomap_binary` to identify "Unknown" voxels and calculate information gain.
* **Lookahead Reset**: Clears old targets upon reactivation to ensure the drone doesn't get stuck in pursuit-to-exploration transition loops.
* **Performance Optimized**: Uses sparse voxel sampling (Step: $res \times 10.0$) and reduced search radii to remain stable on lower-end hardware/VMs.
* **Physical Spin Recovery**: Automatically commands a yaw-spin and forward "squeeze" if pathfinding scores plateau.

---

## 3. Lantern Hunting Package (`lantern_hunting_pkg`)

### Overview

A high-performance C++ implementation of the visual pursuit logic, ported from Python to ensure synchronization with the C++ trajectory pipeline.

### Implementation Details

* **Visual Servoing**: Translates pixel offsets from the semantic camera into 3D translation commands.
* **State Awareness**: Only processes camera data when receiving an `is_enabled_` signal from the State Machine.
* **Post-Hunt Clearance**: Executes a vertical climb and forward burst after "logging" a lantern to clear the zone for the next exploration phase.
* **Completion Signaling**: Publishes to `/hunting_done` once the zone-clearing maneuver is finished.

---

## 4. Trajectory Generator Package (`trajectory_generator_pkg`)

### Overview

The "Math Engine" that converts discrete waypoints (from State Machine or Explorer) into physically feasible curves.

### Topic Interfaces

* **Subscribes to**: `/current_state_est` (Odometry) and `/next_setpoint` (Goal).
* **Publishes to**: `/command/trajectory` (Multi-DOF Setpoints).

---

## 🚀 Usage / Testing

### 1. Build and Refresh

To clear old Python caches and perform a clean C++ build:

```bash
cd ~/Downloads/FinalProjectAutonomousSystem-dev-tuning_cave_exploraton_new/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash

```

In each terminal:

```bash
ros2 launch simulation simulation.launch.py
ros2 launch perception_pkg mapping.launch.py
ros2 run exploration_pkg nbv_explorer_3d_node
ros2 run trajectory_generator_pkg trajectory_generator_node
ros2 run decision_making_pkg state_machine_node
ros2 run lantern_hunting_pkg hunting_node

```



```

Would you like me to help you create a **final zip of this workspace** or generate a **one-click cleanup script** to automate the cache-clearing routine?
