import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys

# Get the directory scripts
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from urdf_gen import generate_urdf

### Launches the epuck into the world room.world (world with only four walls)
def generate_launch_description():    
    world = os.path.join(get_package_share_directory('my_simulation'), 'worlds', 'room.world')

    # Generate bocbot.urdf file with personalized paths
    generate_urdf()

    urdf = os.path.join(get_package_share_directory('my_simulation'), 'models', 'urdf', 'bocbot_gen.urdf')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    spwan_args = '{name: \"bocbot\", xml: \"'  +  xml + '\" }'
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),

    ])