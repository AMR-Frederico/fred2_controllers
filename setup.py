import os
from setuptools import setup
from glob import glob

package_name = 'fred2_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.lib'],
    install_requires=[
        'setuptools',
        'rclpy',
        'transforms3d',  
    ],
    zip_safe=True,
    maintainer='Fre',
    maintainer_email='ubuntu@todo.todo',
    description="The Controllers package provides various controllers for the robot, including a position controller that utilizes a PID algorithm for controlling the robot's position. The primary node in this package is positionController, which is responsible for managing the robot's movement towards a goal position.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'positionController = fred2_controllers.positionController:main',
        ],
    },
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),],
)
