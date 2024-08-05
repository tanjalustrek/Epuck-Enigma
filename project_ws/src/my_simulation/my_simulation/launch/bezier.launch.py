import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Get the directory scripts
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from mesh_gen import generate_meshes
from bezier_gen import generate_bezier
from urdf_gen import generate_urdf

def generate_launch_description():
    # Get the world path
    world_file = os.path.join(get_package_share_directory('my_simulation'), 'worlds', 'room.world')
    
    # Create a mesh and get the longest distance from the centre and generate a .sdf file
    max_distance = generate_meshes()
    generate_bezier()
    # Get .sdf file path
    sdf_file = os.path.join(get_package_share_directory('my_simulation'), 'models', 'bezier', 'model_1.sdf')

    # Generate a bocbot_gen.urdf file with personalized paths and prepare it for use
    generate_urdf()
    urdf = os.path.join(get_package_share_directory('my_simulation'), 'models', 'urdf', 'bocbot_gen.urdf')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    spwan_args = f'{{name: \"bocbot\", xml: \"{xml}\", initial_pose: {{position: {{x: 0, y: 0.5, z: 0.01}}}}}}'

    return LaunchDescription([
        # Start Gazebo with the specified world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawner',
            output='screen',
            arguments=[
                '-entity', 'model',
                '-file', sdf_file,
                '-x', '0', '-y', '0', '-z', '0.05'
            ]
        ),
        ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
        output='screen',
        )
    ])