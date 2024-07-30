from setuptools import setup
import os

package_name = 'my_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['my_simulation/worlds/room.world']),
        ('share/' + package_name + '/launch', ['my_simulation/launch/world.launch.py']),
        ('share/' + package_name + '/launch', ['my_simulation/launch/epuck.launch.py']),
        ('share/' + package_name + '/launch', ['my_simulation/launch/epuck_world.launch.py']),
        ('share/' + package_name + '/meshes', ['my_simulation/meshes/triangular_prism.stl']),
        ('share/' + package_name + '/meshes', ['my_simulation/meshes/left_wheel.STL']),
        ('share/' + package_name + '/meshes', ['my_simulation/meshes/main_body.STL']),
        ('share/' + package_name + '/meshes', ['my_simulation/meshes/right_wheel.STL']),
        ('share/' + package_name + '/models/urdf', ['my_simulation/models/urdf/bocbot.urdf']),
        ('share/' + package_name + '/models/urdf', ['my_simulation/models/urdf/bocbot_gen.urdf']),
        ('share/' + package_name + '/scripts', ['my_simulation/scripts/cuboid_gen.py']),
        ('share/' + package_name + '/scripts', ['my_simulation/scripts/cylinder_gen.py']),
        ('share/' + package_name + '/scripts', ['my_simulation/scripts/triangular_prism_gen.py']),
        ('share/' + package_name + '/scripts', ['my_simulation/scripts/urdf_gen.py']),
        ('lib/' + package_name, ['my_simulation/scripts/epuck_teleop_key_down.py']),
        ('lib/' + package_name, ['my_simulation/scripts/epuck_teleop_key.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romi-lab-2',
    maintainer_email='tanjalustrek@gmail.com',
    description='Spawns random simple objects',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_simulation.my_node:main',
            'epuck_teleop_key = my_simulation.epuck_teleop_key:main',
            'epuck_teleop_key_down = my_simulation.epuck_teleop_key_down:main',
        ],
    },
    package_data={
        'my_simulation': [
            'meshes/triangular_prism.stl',
            'meshes/left_wheel.STL',
            'meshes/main_body.STL',
            'meshes/right_wheel.STL',
            'scripts/cuboid_gen.py',
            'scripts/cylinder_gen.py',
            'scripts/triangular_prism_gen.py',
            'scripts/epuck_teleop_key.py',
            'scripts/epuck_teleop_key_down.py',
            'scripts/urdf_gen.py',
        ],
    },
)