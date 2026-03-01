# Exploration Package

## Overview
The `exploration_pkg` is responsible for autonomous navigation within unknown environments. The main node, `nbv_explorer_3d_node`, implements a **Next Best View (NBV)** algorithm to determine the next position that maximizes the acquisition of new information for the map.

## Key Functionalities
* **3D Lookahead Planning**: Randomly samples 60 candidate points in a forward cone relative to the drone to find the optimal trajectory.
* **Information Gain Scoring**: Evaluates candidate points by counting "Unknown" voxels in the Octomap; points that allow the sensors to see more unmapped areas receive higher scores.
* **Raycasting Safety**: Utilizes Octomap's `castRay` function to verify that there are no physical obstacles between the current position and the target.
* **Spherical Forcefield**: Applies a 5-meter safety radius around candidates to prevent collisions with cave walls.
* **Stuck Recovery (Spin Fix)**: If no valid paths are found, it commands a yaw rotation to force the drone to explore new directions.

## Topic Interfaces

### Subscribed Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/enable_exploration` | `std_msgs/msg/Bool` | Activates the exploration cycle when it receives `True` from the State Machine. |
| `/current_state_est` | `nav_msgs/msg/Odometry` | Provides the current position and orientation for candidate calculation. |
| `/octomap_binary` | `octomap_msgs/msg/Octomap` | The updated 3D map used for scoring and raycasting. |

### Published Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/next_setpoint` | `geometry_msgs/msg/PoseStamped` | Sends the calculated "Next Best View" to the Trajectory Generator. |

## Usage
The node is intended to be activated by the State Machine once the drone reaches the cave entrance.