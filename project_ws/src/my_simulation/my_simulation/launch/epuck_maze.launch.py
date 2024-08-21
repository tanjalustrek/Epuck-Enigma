import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys
import random

# Get the scripts directory
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from urdf_gen import generate_urdf
from maze_gen import row_4
from maze_gen import row_3
from maze_gen import row_2
from maze_gen import row_1

def generate_launch_description():
    '''Launches an epuck into a maze'''
    # Get the world path
    world = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'worlds', 'maze_gen.world')
    
    # Modity the maze
    row_4(world)
    row_3(world)
    row_2(world)
    row_1(world)

    # Generate bocbot_gen.urdf file with personalized paths and prepare it for use
    generate_urdf()
    urdf = os.path.join(get_package_share_directory('my_simulation'), 'models', 'urdf', 'bocbot_gen.urdf')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    random_x = random.uniform(-0.6, 0.6)
    spwan_args = f'{{name: \"bocbot\", xml: \"{xml}\", initial_pose: {{position: {{x: {random_x}, y: 1.25, z: 0}}}}}}'
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),
    ])