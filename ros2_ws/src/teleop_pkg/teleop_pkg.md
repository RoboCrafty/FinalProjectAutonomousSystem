# Teleop Package

## Overview
The `teleop_pkg` provides a manual flight interface for the quadrotor. It allows a human operator to control the drone using a keyboard while maintaining flight stability through smoothed velocity setpoints.

## Key Functionalities
* **Flight Computer Emulation**: Smoothes raw keyboard inputs into physically feasible acceleration and velocity ramps ($4.0$ m/s max speed).
* **Automatic Synchronization**: Upon startup, the node waits for `/true_pose` to synchronize its internal target with the drone's actual position, preventing sudden jumps.
* **Intelligent Yaw Control**: Converts body-frame "Forward/Left" commands into global "World-frame" coordinates based on the drone's current rotation.
* **Safety Exit**: Includes terminal cleanup logic to restore keyboard settings and stop movement upon node shutdown.

## Keyboard Mapping
| Key | Flight Command |
| :--- | :--- |
| **W / S** | Move Forward / Backward |
| **A / D** | Move Left / Right |
| **Space / Z** | Ascend / Descend |
| **Q / E** | Yaw Rotate Left / Right |

## Topic Interfaces

### Subscribed Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/true_pose` | `geometry_msgs/msg/PoseStamped` | Used for one-time initialization of the target state to match the drone's actual position. |

### Published Topics
| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | Publishes high-frequency (50Hz) trajectory setpoints to the geometric controller. |

## Usage
```bash
ros2 run teleop_pkg teleop_node