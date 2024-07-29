import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import random
import math

# Get the directory containing cuboid_gen.py and cylinder_gen.py
scripts_dir = os.path.join(get_package_share_directory('my_simulation'), 'scripts')
sys.path.append(scripts_dir)

from cuboid_gen import generate_cuboid
from cuboid_gen import get_edges
from cylinder_gen import generate_cylinder
from cylinder_gen import get_radius
from triangular_prism_gen import generate_triangular_prism
from triangular_prism_gen import get_side_length_scale

### Launches random objects inside a room in the room.world world
def generate_launch_description():
    world_file = os.path.join(get_package_share_directory('my_simulation'), 'worlds', 'room.world')
    my_objects = ['cuboid', 'cylinder', 'triangular_prism']

    # Set parameters
    buffer = 0.071 # Epuck's diameter
    max_retries = 10

    # Define the range for the number of objects to spawn
    min_objects, max_objects = 10, 15
    # Generate a random number of objects to spawn
    num_objects = random.randint(min_objects, max_objects)
    # Generate object files
    generate_cuboid(num_objects)
    generate_cylinder(num_objects)
    generate_triangular_prism(num_objects)

    # Create a list to store the Node actions and positions of objects
    nodes = []
    spawned_positions = []

    max_overall = 0
    # Spawn the objects at random positions
    for i in range(num_objects):
        # Set paths
        cuboid_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cuboid', f'model_{i+1}.sdf')
        cylinder_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'cylinder', f'model_{i+1}.sdf')
        triangular_prism_path = os.path.join(os.path.abspath('src'), 'my_simulation', 'my_simulation', 'models', 'triangular_prism', f'model_{i+1}.sdf')

        # Choose the object
        my_object = random.choice(my_objects)

        # Calculate the 'radius'
        if my_object == 'cuboid':
            max_size = (math.sqrt((get_edges(cuboid_path)[0])**2 + (get_edges(cuboid_path)[1])**2))/2
        elif my_object == 'cylinder':
            max_size = get_radius(cylinder_path)
        elif my_object == 'triangular_prism':
            max_size = math.sqrt(3)/3 * 4 * get_side_length_scale(triangular_prism_path)

        # Set the minimum distance between objects
        if max_size > max_overall:
            max_overall = max_size
        min_distance = max_overall + max_size + buffer

        # Define the range for the random location
        min_x, max_x = -0.75 + max_size, 0.65 - max_size
        min_y, max_y = -1.34 + max_size, 1.44 - max_size

        # Check if the random position is at least min_distance away from all other spawned positions
        # If not, try again with a new random position
        is_valid_position = False
        retry_count = -1
        while (not is_valid_position) and (retry_count < max_retries):
            # Choose random position
            random_x = random.uniform(min_x, max_x)
            random_y = random.uniform(min_y, max_y)
            random_position = (random_x, random_y)

            is_valid_position = True
            for position in spawned_positions:
                distance = math.sqrt((random_x - position[0])**2 + (random_y - position[1])**2)
                if distance < min_distance:
                    is_valid_position = False
                    break

            retry_count += 1

        if is_valid_position:
            # Randomly remove objects
            if random.uniform(0, 1) < 0.1:
                continue

            # Add nodes
            if my_object == 'cuboid':
                nodes.append(
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name=f'spawner_{i+1}',
                        output='screen',
                        arguments=[
                            '-entity', f'object_{i+1}',
                            '-file', cuboid_path,
                            '-x', str(random_x),
                            '-y', str(random_y),
                            '-z', '0.25'
                        ]
                    )
                )
            elif my_object == 'cylinder':
                nodes.append(
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name=f'spawner_{i+1}',
                        output='screen',
                        arguments=[
                            '-entity', f'object_{i+1}',
                            '-file', cylinder_path,
                            '-x', str(random_x),
                            '-y', str(random_y),
                            '-z', '0.25'
                        ]
                    )
                )
            elif my_object == 'triangular_prism':
                nodes.append(
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        name=f'spawner_{i+1}',
                        output='screen',
                        arguments=[
                            '-entity', f'object_{i+1}',
                            '-file', triangular_prism_path,
                            '-x', str(random_x),
                            '-y', str(random_y),
                            '-z', '0.25'
                        ]
                    )
                )
            # Save position of the object
            spawned_positions.append(random_position)

    # Execute
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', scripts_dir + ':' + os.environ.get('PYTHONPATH', '')),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),
        *nodes
    ])