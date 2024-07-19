# Spawn random objects in Gazebo

In this simulation a random number of random sized and oriented objects are spawned inside a room.

The simulation was built with **Gazebo 11.14.0** and **ROS 2 foxy** on **Ubuntu 20.04**

Instructions to run on Ubuntu 20.04:
1. Make sure you have installed the right versions of Gazebo and ROS 2.
2. Open ***project_ws*** in the terminal.
3. Run:
    - `colcon build`
    - `source install/setup.bash`
    - `export GAZEBO_MODEL_PATH=/.../project_ws/src/my_simulation/my_simulation/models:$GAZEBO_MODEL_PATH` where you replace ... with the location of ***project_ws*** on your computer.
    - `ros2 launch my_simulation project.launch.py`
4. Gazebo should start and objects should appear inside the room.