from setuptools import setup
from glob import glob
import os

package_name = 'my_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('my_simulation/worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('my_simulation/launch/*.launch.py')),
        (os.path.join('share', package_name, 'models', 'urdf'), glob('my_simulation/models/urdf/*.urdf')),
        (os.path.join('share', package_name, 'scripts'), glob('my_simulation/scripts/*.py')),
        ('lib/' + package_name, ['my_simulation/scripts/epuck_teleop_key.py']),
        ('lib/' + package_name, ['my_simulation/scripts/epuck_teleop_key_two.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romi-lab-2',
    maintainer_email='tanjalustrek@gmail.com',
    description='Creates different random worlds and can spawn an epuck in them',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'epuck_teleop_key = my_simulation.epuck_teleop_key:main',
            'epuck_teleop_key_two = my_simulation.epuck_teleop_key_two:main',
        ],
    },
)