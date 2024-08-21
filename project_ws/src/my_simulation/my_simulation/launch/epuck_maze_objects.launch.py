import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import sys
from launch_ros.actions import Node

# Get the scripts directory
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from urdf_gen import generate_urdf
from fixed_triangular_prism_gen import generate_fixed_triangular_prism

def generate_launch_description():
    '''Launches an epuck into a maze'''
    # Generate the .sdf file for the triangular_prism
    generate_fixed_triangular_prism()

    # Get the world and cube path
    world_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'worlds', 'maze.world')
    cube_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cuboid', 'cube.sdf')
    cylinder_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cylinder', 'cylinder.sdf')
    triangular_prism_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'triangular_prism', 'triangular_prism.sdf')

    # Generate bocbot_gen.urdf file with personalized paths and prepare it for use
    generate_urdf()
    urdf = os.path.join(get_package_share_directory('my_simulation'), 'models', 'urdf', 'bocbot_gen.urdf')
    xml = open(urdf, 'r').read()
    xml = xml.replace('"', '\\"')
    spwan_args_1 = f'{{name: \"epuck1\", xml: \"{xml}\", robot_namespace: \"epuck1\", initial_pose: {{position: {{x: 0.3, y: 1.25, z: 0}}}}}}'
    spwan_args_2 = f'{{name: \"epuck2\", xml: \"{xml}\", robot_namespace: \"epuck2\", initial_pose: {{position: {{x: -0.6, y: 0.2, z: 0}}}}}}'

    # Add objects
    coordinates_cube = [[0.3, 1.15], [-0.4, -0.1], [-0.2, -0.8]]
    cubes = []
    for i in range(len(coordinates_cube)):
        cubes.append(
            Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='cube',
            output='screen',
            arguments=[
                '-entity', f'cube_{i+1}',
                '-file', cube_path,
                '-x', str(coordinates_cube[i][0]),
                '-y', str(coordinates_cube[i][1]),
                '-z', '0.025'
            ]
        ),
        )

    coordinates_cylinder = [[0.1, -0.7], [0.4, 0.1], [0.2, 0.8]]
    cylinders = []
    for i in range(len(coordinates_cylinder)):
        cylinders.append(
            Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='cylinder',
            output='screen',
            arguments=[
                '-entity', f'cylinder_{i+1}',
                '-file', cylinder_path,
                '-x', str(coordinates_cylinder[i][0]),
                '-y', str(coordinates_cylinder[i][1]),
                '-z', '0.025'
            ]
        ),
        )

    coordinates_triangular_prism = [[0.5, -0.2], [-0.5, 0.7], [-0.1, 0.5]]
    triangular_prisms = []
    for i in range(len(coordinates_triangular_prism)):
        triangular_prisms.append(
            Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='cylinder',
            output='screen',
            arguments=[
                '-entity', f'triangular_prism_{i+1}',
                '-file', triangular_prism_path,
                '-x', str(coordinates_triangular_prism[i][0]),
                '-y', str(coordinates_triangular_prism[i][1]),
                '-z', '0.025'
            ]
        ),
        )
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args_1],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args_2],
            output='screen'),
            *cubes,
            *cylinders,
            *triangular_prisms,
    ])