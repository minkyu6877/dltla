import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cargo_fleet_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kminh0712',
    maintainer_email='kminh0712@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_manager = cargo_fleet_manager.fleet_manager:main',
            'mission_manager = cargo_fleet_manager.mission_manager:main',
            'qr_reader = cargo_fleet_manager.qr_reader:main',
            'uwb_simulator = cargo_fleet_manager.uwb_simulator:main',
            'kinematic_visualizer = cargo_fleet_manager.kinematic_visualizer:main',
            'coordinate_console = cargo_fleet_manager.coordinate_console:main',
        ],
    },
)
