# Execution and Launch Procedures

## 1. Workspace Preparation

Before running the mission, you must compile all packages and source the environment. This ensures that the custom message types and nodes are recognized by the ROS 2 middleware.

```bash
# Navigate to the workspace root
cd ~/ros2_ws

# Build the project using symlink-install to allow immediate launch file updates
colcon build --symlink-install

# Source the overlay to update the environment
source install/setup.bash

```

## 2. Terminal 1: Simulation Startup

The simulation must be started first. This terminal runs the Unity-based environment which provides the Octomap data, camera feeds, and drone physics.

```bash
# Start the simulation environment
ros2 launch simulation simulation.launch.py

```

> **Note:** Wait for the simulation to fully load and for the drone to appear at the starting position before proceeding to the next step.

## 3. Terminal 2: Autonomous Mission Startup

Once the simulation is active, open a new terminal to start the decision-making logic and navigation stack. This unified launch file triggers the explorer, hunter, and state machine.

```bash
# Source the environment in the new terminal
source ~/ros2_ws/install/setup.bash

# Launch the complete autonomous logic
ros2 launch decision_making_pkg mission.launch.py

```

---