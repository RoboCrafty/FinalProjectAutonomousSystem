Step 1: copy and paste manual_pilot.cpp into ros2_ws/src/controller_pkg/src
	copy and paste lantern_tracker.py into ros2_ws/src/simulation/scripts

Step 2: Build the controller library again:
colcon build
source install/setup.bash

Step 3: run the following terminals with following commands

Terminal 1: ros2 launch simulation simulation.launch.py (Starts Unity, create a new folder called unity_sim and add the files from the simuation inside).

Terminal 2: ros2 run controller_pkg manual_pilot (Starts the script above).

Terminal 3: ros2 run simulation lantern_tracker.py 

Terminal 4: rviz2 (add camera feed from drone: semantics camera/image_raw/image & /realsensedepth/image/image)

