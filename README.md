# Epuck Enigma

This is a dynamic simulation environment built with **ROS 2 foxy** and **Gazebo 11.14.0** on **Ubuntu 20.04**. This project allows for the creation and spawning of random mazes and objects, providing a versatile platform for testing and development. It also enables adding epucks to interact with the created worlds.  

Structure:  
 - workspace: ***project_ws***,
 - package: ***my_simulation***,
 - worlds:  
  - ```room.world```: world with a simple rectangular room,
  - ```maze.world```: world with a fixed maze,
  - ```maze_gen.world```: world with a random maze,
 - launch files:  
  - ```objects.launch.py```: launches a random number of random static objects in ```room.world```,
  - ```epuck_objects.launch.py```: the same as previous but also launches an Epuck in the world,
  - ```epuck_objects_bezier.launch.py```: the same as previous but also launches randomly shaped movable objects,
  - ```epuck_bezier.launch.py```: launches an Epuck along with a randomly shaped movable object,
  - ```epuck_maze.launch.py```: launches an Epuck in ```maze_gen.world```,
  - ```epuck_maze_objects.launch.py```: launches an Epuck in ```maze.world``` along with some movable objects,
  - ```epuck.launch.py```: launches an Epuck in ```room.world```.
 - 

Instructions to run on Ubuntu 20.04:
1. Make sure you have installed the right versions of Gazebo and ROS 2.
2. Open the terminal and navigate to the location where you want the cloned repository. Then copy the HTTPS link for this repository and run ```git clone HTTPS_LINK```.
3. Open ***project_ws*** in the terminal.
4. Run:
    - ```
      colcon build
      ```
    - ```
      source install/setup.bash
      ```
    - ```
      export GAZEBO_MODEL_PATH=/.../project_ws/src/my_simulation/my_simulation/models:$GAZEBO_MODEL_PATH
      ```
      where you replace ... with the location of ***project_ws*** on your computer.
    - Depending on what simulation you want to run, choose the appropriate launch file. If you just want to spawn the random objects or just the epuck choose ```world.launch.py``` or ```epuck.launch.py``` respectively, otherwise use ```epuck_world.launch.py```.
      ```
      ros2 launch my_simulation CHOSEN_LAUNCH_FILE
      ```
5. The simulation should start.
6. If you want to move your epuck open ***project_ws*** in a new terminal while the simulation is running. Then choose either ```epuck_teleop_key.py``` or ```epuck_teleop_key_down.py``` and run the following lines.
   - ```
     source install/setup.bash
     ```
   - ```
     ros2 run my_simulation CHOSEN_PYTHON_FILE
     ```
   Now you should be able to move the epuck.

