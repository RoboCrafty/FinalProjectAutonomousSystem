Step 1: Download the new repo given by the Prof from gitlab

Step 2: Create a new directory called unity_sim in ros2_ws/src/simulation. Copy and paste the 3 simulations file given by the Prof. Keep in mind if u use LQ or HQ, u have to rename the simulation file names. (Simulation_LQ_Data to Simulation_Data)

Step 3: Update the cmake file in ros2_ws/src/simulation with the one I gave

Step 4: Create a new directory called scripts in

Step 5: Copy and paste manual_pilot.cpp into ros2_ws/src/controller_pkg/src

Step 6: Update the cmake file in ros2_ws/src/controller_pkg with the one I gave

Step 7: source /opt/ros/jazzy/setup.bash 
	colcon build 
	source install/setup.bash

Step 8: run the following terminals with following commands. (Dont forget to source ros2)

Terminal 1: 
	source install/setup.bash
        ros2 launch simulation simulation.launch.py
        
Terminal 2: 
	source install/setup.bash
	ros2 run simulation lantern_tracker.py 

Terminal 3: 
	source install/setup.bash
	ros2 run controller_pkg manual_pilot

Terminal 4: rviz2 (add camera feed from drone: semantics camera/image_raw/image & /realsensedepth/image/image)



