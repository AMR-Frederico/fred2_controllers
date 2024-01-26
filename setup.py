from setuptools import setup

package_name = 'fred2_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',
        'transforms3d',  # Add other dependencies as needed
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='leticiarp2000@hotmail.com',
    description='ROS 2 package for position control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'positionController = fred2_controllers.positionController:main',
        ],
    },
)
