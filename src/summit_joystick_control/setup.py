from setuptools import setup
import os
from glob import glob

package_name = 'summit_joystick_control'


setup(
    name=package_name,
    version='0.0.0',
    py_modules=['joystick_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),  
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
            'joystick_controller = joystick_controller:main',   ## for manual control with joystick
            'linefollow_robot1 = line_follower.LineFollowRobot1:main',
            'linefollow_robot2 = line_follower.LineFollowRobot2:main',  # new sharp turn logic
        ],
    },
)
