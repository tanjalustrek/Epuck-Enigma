import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys
import math
import random

# Get the scripts directory
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from urdf_gen import generate_urdf

def generate_launch_description():
    '''Launches an epuck into a world with only four walls'''
    # Get the world path
    world = os.path.join(get_package_share_directory('my_simulation'), 'worlds', 'room.world')

    # Generate bocbot_gen.urdf file with personalized paths and prepare it for use
    generate_urdf()
    urdf = os.path.join(get_package_share_directory('my_simulation'), 'models', 'urdf', 'bocbot_gen.urdf')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    epuck_node = []

    # Define the position
    buffer = 0.071
    random_x = random.uniform(-0.75 + buffer/2, 0.65 - buffer/2)
    random_y = random.uniform(-1.34 + buffer/2, 1.44 - buffer/2)

    # Define the orientation
    yaw = random.uniform(-math.pi, math.pi)
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)

    # Update the spawn_args to include orientation
    spwan_args = f'{{name: \"bocbot\", xml: \"{xml}\", initial_pose: {{position: {{x: {random_x}, y: {random_y}, z: 0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}'

    epuck_node.append(ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
        output='screen',
    ))
    
    # Execute
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        *epuck_node,
    ])