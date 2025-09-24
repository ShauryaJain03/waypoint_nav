from setuptools import find_packages, setup
import glob as glob
import os
package_name = 'waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/smooth_waypoint.launch.py']), 
        ('share/' + package_name + '/rviz', ['rviz/smooth_trajectory.rviz']), 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaurya',
    maintainer_email='jainshaurya.sj@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "smoother=waypoint_nav.smoother:main",
            "controller=waypoint_nav.controller:main",
            "obstacle_avoid=waypoint_nav.obstacle_avoid:main",
        ],
    },
)
