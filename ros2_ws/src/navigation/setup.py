import glob
from setuptools import setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed',
    maintainer_email='ahmed@example.com',
    description='Waypoint navigation node for the autonomous RC car',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_manager = navigation.waypoint_manager_node:main',
            'cmd_vel_gate = navigation.cmd_vel_gate_node:main',
            'local_planner = navigation.local_planner_node:main',
            'safety_stop = navigation.safety_stop_node:main',
        ],
    },
)
