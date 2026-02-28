# Autonomous System Project 2026 - Cave Exploration Drone

This workspace contains the ROS 2 (Jazzy) packages necessary to simulate and control a quadrotor drone for an autonomous cave exploration mission in the Sub-Terrain Challenge. The architecture is highly modular, separating high-level mission state logic, 3D perception, path planning, and low-level control.

---

## 🏗️ Architecture Overview

The system is designed as a continuous pipeline:
**Mission Control ➡️ Path Planning ➡️ Trajectory Generator ➡️ Controller ➡️ Simulation**
*(with Odometry and Octomap running as feedback loops).*

---

## 1. Mission Control Package (`mission_control_pkg`)

### Overview

The `mission_control_pkg` is the "Brain" (State Machine) of the drone. It manages the phases of the mission (e.g., "Fly to Cave Entrance", "Explore Cave", "Return Home"). It does not compute physics or collision avoidance; it merely dispatches high-level target coordinates to the navigation nodes.

### Implementation Details

* **Phase-Based State Machine:** Implemented as a C++ Node using a timer-driven state machine.
* **Phase 0 (Jumpstart):** To bypass the Controller's idle safety state, this node first broadcasts a single static `MultiDOFJointTrajectory` point to arm the motors and achieve a stable hover at Z=10.0.
* **Phase 1 (Transit):** After a 4-second stabilization delay, it broadcasts a hardcoded array of 16 "stepping stone" waypoints to guide the drone to the cave entrance.
* **QoS Handling:** Uses `transient_local` QoS to guarantee the path is received by subscribers even if sent immediately upon startup.

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/current_state_est` | `nav_msgs/msg/Odometry` | Used to verify if the drone has reached its target state before advancing to the next mission phase. |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/planned_path` | `nav_msgs/msg/Path` | The array of high-level waypoints representing the mission goal. |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | Used specifically during Phase 0 to initialize the controller. |

---

## 2. Path Planning Package (`path_planning_pkg`)

### Overview

The `path_planning_pkg` is the primary navigation component. It acts as the bridge between perception (the 3D map) and the controller. This node utilizes a custom 3D **A* (A-Star) graph-search algorithm** to find a collision-free path to a target destination provided by Mission Control.

### Implementation Details

* **3D A* Search:** Explores the 26-connected neighbor space in a 3D grid.
* **Volumetric Collision Checking:** Checks a 0.4m radius spherical bounding box around the drone to ensure the physical quadrotor body does not hit walls.
* **Unknown Space Handling:** Treats unknown/unmapped space as obstacles to guarantee the drone only flies into visually verified safe zones.
* **Trajectory Smoothing:** Subsamples the raw A* path and feeds it to the `trajectory_generator_pkg`.

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/current_state_est` | `nav_msgs/msg/Odometry` | The starting point for the A* search. |
| `/octomap_binary` | `octomap_msgs/msg/Octomap` | The boolean representation of the 3D map for collision checking. |
| `/mission_goal` | `geometry_msgs/msg/PoseStamped` | The target destination from Mission Control. |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/raw_astar_path` | `nav_msgs/msg/Path` | The discrete, jagged sequence of 3D coordinates output directly by the A* algorithm (sent to Trajectory Generator). |

---

## 3. Trajectory Generator Package (`trajectory_generator_pkg`)

### Overview

The `trajectory_generator_pkg` is the "Math Engine". It takes raw, jagged paths and converts them into physically flyable 4D polynomial curves.

### Implementation Details

* **Polynomial Optimization:** Formulates a Minimum-Snap non-linear optimization problem using the `mav_trajectory_generation` library.
* **Real-Time Feeder Logic:** To prevent the controller from ignoring large trajectory buffers, this node samples the optimized curve into discrete "breadcrumbs" every 20ms.
* **50Hz Playback:** A playback timer feeds exactly one `MultiDOFJointTrajectoryPoint` at a time to the controller, ensuring smooth and continuous movement along the entire path.
* **Numerical Stability:** Requires multiple waypoints (stepping stones) to prevent matrix singularity crashes and stack-smashing errors during long-distance calculations.

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/planned_path` / `/raw_astar_path` | `nav_msgs/msg/Path` | Receives raw waypoints to be smoothed. Uses `transient_local` QoS. |
| `/current_state_est` | `nav_msgs/msg/Odometry` | Anchors the first polynomial segment to the drone's true current velocity. |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | Streams discrete, single-point trajectory commands to the controller at 50Hz. |

---

## 4. Perception Package (`octomap_pkg` - *To Be Implemented*)

### Overview

The `octomap_pkg` serves as the "Eyes" of the drone. It consumes 3D point cloud and depth data from the simulated RealSense camera and builds a persistent 3D occupancy grid (Voxel map) of the cave as the drone flies.

### Implementation Details

* Utilizes the `octomap_server` library.
* Will likely require a `depth_image_proc` node to convert the raw depth images (`/realsense/depth/image`) into `PointCloud2` messages.
* Maintains a real-time TF tree to project camera coordinates into the static `world` frame.

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/realsense/depth/image` | `sensor_msgs/msg/Image` | Raw depth data from the Unity simulation. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform data to align the camera with the world. |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/octomap_binary` | `octomap_msgs/msg/Octomap` | The 3D collision map consumed by the A* Planner. |

---

## 5. Lower-Level Packages (*Provided by University*)

### `controller_pkg` (The Reflexes)

Reads the trajectory commands and calculates rotor speeds to maintain the desired state.

* **Subscribes to:** `/command/trajectory`, `/current_state_est`
* **Publishes to:** `/rotor_speed_cmds` (`mav_msgs/msg/Actuators`)

### `simulation` (The Unity World)

A bridge between ROS 2 and Unity. Simulates drone physics, collisions, and sensor output.

* **Subscribes to:** `/rotor_speed_cmds`
* **Publishes to:** `/current_state_est`, `/realsense/depth/image`, `/realsense/rgb/image_raw`

---

## 🚀 Usage / Testing

The system is launched via a combined launch file and a separate mission trigger.

**1. Launch the Simulation, Controller, and Trajectory Generator:**

```bash
source install/setup.bash
ros2 launch simulation simulation.launch.py

```

---

**Would you like me to now generate the initial `hunter_node.cpp` for your new `lantern_hunting_pkg` so you can begin testing Phase 2?**
