# 🛰️ Autonomous Drone Cave Exploration System

This repository contains a **ROS 2 Jazzy–based autonomous drone exploration system** for navigating and mapping a cave environment using **OctoMap**.

---

# 📦 Prerequisites

## 1️⃣ Install ROS 2 Jazzy

Make sure ROS 2 Jazzy is installed:

👉 https://docs.ros.org/en/jazzy/Installation.html

---

## 2️⃣ Install Required System Dependencies

The project depends on several ROS 2 packages for mapping, perception, TF, and point cloud processing.

Run:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-octomap \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap-msgs \
    ros-jazzy-depthimage-to-pointcloud2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    python3-numpy
```

---

# 🛠️ Build Instructions

⚠️ The repository already contains a `src` folder, so it acts as a complete ROS 2 workspace.

## 1️⃣ Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
```

## 2️⃣ Clone the Repository

```bash
git clone https://github.com/RoboCrafty/FinalProjectAutonomousSystem.git && cd FinalProjectAutonomousSystem
```

## 3️⃣ Install Remaining Dependencies via rosdep

```bash
rosdep install --from-paths src -y --ignore-src
```

## 4️⃣ Build the Workspace

```bash
colcon build
```

---

# 🎮 Simulator Setup (Unity)

Two simulator versions are available:

- **Low-Quality (LQ)** – Recommended for less powerful machines  
- **High-Quality (HQ)** – Better visuals

## Setup Steps

1. Choose either the **LQ** or **HQ** simulator.
2. Copy the simulator executable **and its corresponding data folder** into:

```
install/simulation/lib/simulation/
```

3. Rename them to the default names so the launch file can detect them:

- Executable → `Simulation.x86_64`
- Data folder → `Simulation_Data`

⚠️ Both names must match exactly.

---

# 🚀 Running the Project

You need **two terminals**.

⚠️ In BOTH terminals, from the root of your repository:

```bash
source install/setup.bash
```

---

## 🖥️ Terminal 1 – Launch Simulation

Wait until the Unity window fully loads before continuing.

```bash
ros2 launch simulation simulation.launch.py
```

---

## 🧠 Terminal 2 – Launch Exploration Mission

```bash
ros2 launch decision_making_pkg mission.launch.py
```

---

# ✅ Summary

1. Install ROS 2 Jazzy  
2. Install dependencies  
3. Clone repo  
4. `rosdep install`  
5. `colcon build`  
6. Copy & rename Unity simulator  
7. Launch simulation (Terminal 1)  
8. Launch mission (Terminal 2)

---

🛰️ Your autonomous drone should now begin exploring and building an OctoMap of the cave environment.
