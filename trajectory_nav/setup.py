from setuptools import setup
import os
from glob import glob

package_name = 'trajectory_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Include RViz config if present
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS2 trajectory navigation system with smooth path generation and pure pursuit control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator = trajectory_nav.trajectory_generator:main',
            'trajectory_controller = trajectory_nav.trajectory_controller:main',
            'trajectory_monitor = trajectory_nav.trajectory_monitor:main',
        ],
    },
)


