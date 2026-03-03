# Technical Documentation: `exploration_pkg`

## 1. Node Architecture: `frontier_explorer.cpp`

The `frontier_explorer` node is the autonomous core of the system. It implements a **Topological Graph-based Frontier Exploration** strategy. This node maintains a spatial memory of the environment to ensure a guaranteed, collision-free return path (backtracking) even in complex cave networks.

### 1.1 Frontier Detection

The drone identifies unmapped areas by searching the Octomap for **Frontier Voxels**.

* **Definition:** A voxel is classified as a frontier if it is labeled as "Free" but is adjacent to at least one "Unknown" (NULL) voxel.
* **Optimization:** To maintain high real-time performance, the search is restricted to a `LOCAL_BOX_SIZE` (60.0m) around the drone. This prevents the CPU from processing the entire global map every cycle.

### 1.2 Spatial Clustering Algorithm

Raw frontier voxels are disorganized. To create actionable navigation targets, the node implements a **Nearest-Neighbor Centroid Clustering** algorithm:

1. **Sorting:** Voxels are sorted by their X-coordinate to provide a stable baseline for grouping.
2. **Grouping:** The algorithm iterates through voxels; if a voxel is within `CLUSTER_DIST_THRESHOLD` (1.2m) of an existing cluster, it joins it. Otherwise, a new cluster is initialized.
3. **Centroid Calculation:** For each group, the algorithm computes the 3D average (mean) of all voxel positions. This centroid becomes the potential flight target.

---

## 2. Advanced Exploration Strategy

### 2.1 Dynamic Threshold Reduction

The exploration is governed by a "negotiation" logic that ensures the drone doesn't give up in narrow passages.

* **The Logic:** The search starts looking for large, high-confidence clusters (`MIN_CLUSTER_SIZE = 150`).
* **Adaptive Scaling:** If no large clusters are found or reachable, the algorithm enters a `while` loop, reducing the required voxel count by **30** in each iteration until it reaches the `ABSOLUTE_MIN_SIZE`.
* **Outcome:** This allows the drone to prioritize wide tunnels while still being capable of exploring tight crevices.

### 2.2 Heading Bias & Scoring

To prevent erratic 180-degree turns and "jittery" behavior, we apply a mathematical weight to candidate clusters:

* **Vector Analysis:** We calculate the dot product between the drone's current progress vector and the vector toward the candidate cluster.
* **The Multiplier:** If the drone is moving toward a cluster (angle < 90°), the cluster's score is multiplied by `HEADING_BIAS`.
* **Result:** The drone effectively "prefers" to continue going deeper into the cave rather than doubling back for small missed spots.

---

## 3. Navigation & Safety Algorithms

### 3.1 The Forbidden Radius Guard

To solve the problem of "Local Oscillations," we implemented a `FORBIDDEN_RADIUS` (8.0m).

* **Mechanism:** Any frontier detected within 8 meters of *any* existing node in the navigation graph is ignored.
* **Purpose:** This forces the drone to seek **new** territory and maintains forward momentum, preventing the drone from getting stuck investigating small mapping gaps in areas it has already visited.

### 3.2 Passive Logging Mode (Handover Safety)

When the high-level State Machine takes control (`is_active_ = false`), the explorer enters **Passive Mode**.

* **Line-of-Sight Monitoring:** Every 5 meters of travel, the node performs an Octomap raycast (`isPathClear`) to the last graph node.
* **Corner Stamping:** If the drone turns a corner and the path is blocked by a wall, the node immediately drops a **Safety Node** at the last known clear position (`prev_pos_`).
* **Significance:** This ensures that the backtracking chain is never broken, even if the drone is moved manually or by a different logic block.

---

## 4. Graph Logic & Backtracking

### 4.1 Graph Construction

The navigation graph is a tree-structure where each `GraphNode` contains a 3D position and a `parent_id`.

* **Discovery:** Nodes are created whenever a frontier is reached.
* **Backtracking:** When no frontiers remain, the state switches to `BACKTRACKING`. The drone pops nodes from the stack and follows the parent IDs back to the entrance.
* **Re-Exploration:** If a new frontier is discovered during the return trip, the system can instantly switch back to `EXPLORING`.

---

## 5. Communication Interface

| Topic | Type | Direction | Brief Explanation |
| --- | --- | --- | --- |
| `/current_state_est` | `nav_msgs/msg/Odometry` | **Sub** | Drone position for graph building. |
| `/octomap_binary` | `octomap_msgs/msg/Octomap` | **Sub** | Global map for frontier and collision checks. |
| `/enable_exploration` | `std_msgs/msg/Bool` | **Sub** | Activation toggle from the State Machine. |
| `/next_setpoint` | `geometry_msgs/msg/PoseStamped` | **Pub** | Target sent to the trajectory generator. |
| `/exploration_complete` | `std_msgs/msg/Bool` | **Pub** | Finish signal sent back to State Machine. |

---

## 6. Build and Usage

```bash
# Build instructions
colcon build --packages-select exploration_pkg
source install/setup.bash

# Run the node
ros2 run exploration_pkg frontier_explorer_node

```

---

*This documentation was prepared for the Autonomous Systems Winter Semester 2025 Group Project.*
