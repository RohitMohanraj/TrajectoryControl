from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trajectory_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Simplified trajectory tracking controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'test_scenarios_node = trajectory_controller.test_scenarios:main',
            'path_smoother_node = trajectory_controller.path_smoother:main',
            'trajectory_generator_node = trajectory_controller.trajectory_generator:main',
            'pure_pursuit_node = trajectory_controller.pure_pursuit_controller:main',
        ],
    },
)
