from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'summit_joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required package metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajpurkar',
    maintainer_email='sahilrajpurkar1998@gmail.com',
    description='Joystick control for Summit XL robot in Isaac Sim using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_controller = summit_joystick_control.joystick_controller:main',
        ],
    },
)
