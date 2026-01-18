from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zli14z',
    maintainer_email='zli14z@todo.todo',
    description='Turtle Tracker con Service E3 E4 y Action E6',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts':  [
            'explorer_service_node = turtle_tracker.explorer_service_node:main',
            'service_client_node = turtle_tracker.service_client_node:main',
            'explorer_action_node = turtle_tracker.explorer_action_node:main',
            'action_client_node = turtle_tracker.action_client_node:main',
        ],
    },
)