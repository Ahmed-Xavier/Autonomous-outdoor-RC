import glob
from setuptools import setup

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Vehicle control nodes for the autonomous outdoor RC',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_controller = vehicle_control.vehicle_controller_node:main',
            'vehicle_state = vehicle_control.vehicle_state_node:main',
        ],
    },
)
