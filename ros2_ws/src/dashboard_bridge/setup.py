import glob
from setuptools import setup

package_name = 'dashboard_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Simulation telemetry bridge for the e-ink dashboard',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'track_progress = dashboard_bridge.track_progress:main',
            'fake_vehicle_status = dashboard_bridge.fake_vehicle_status:main',
        ],
    },
)
