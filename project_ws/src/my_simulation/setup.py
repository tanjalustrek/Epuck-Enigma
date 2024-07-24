from setuptools import setup
import os

package_name = 'my_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['my_simulation/launch/project.launch.py']),
        ('share/' + package_name + '/worlds', ['my_simulation/worlds/room.world']),
        ('share/' + package_name + '/generator', ['my_simulation/generator/cuboid_gen.py']),
        ('share/' + package_name + '/generator', ['my_simulation/generator/cylinder_gen.py']),
        ('share/' + package_name + '/generator', ['my_simulation/generator/triangular_prism_gen.py']),
        ('share/' + package_name + '/models/triangular_prism/meshes', ['my_simulation/models/triangular_prism/meshes/triangular_prism.stl']),
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
        ],
    },
    package_data={
        'my_simulation': [
            'generator/cuboid_gen.py',
            'generator/cylinder_gen.py',
            'generator/triangular_prism_gen.py',
            'models/triangular_prism/meshes/triangular_prism.stl',
        ],
    },
)