import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys

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
    spwan_args = '{name: \"bocbot\", xml: \"'  +  xml + '\" }'
    
    # Execute
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),

    ])