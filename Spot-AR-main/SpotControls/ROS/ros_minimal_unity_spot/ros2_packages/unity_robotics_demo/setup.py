import glob
import os

from setuptools import setup

package_name = 'unity_robotics_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Unity Robotics',
    maintainer_email='unity-robotics@unity3d.com',
    description='ROS2 Unity Integration Testing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_service = unity_robotics_demo.position_service:main',
            'joints_subscriber = unity_robotics_demo.joints_subscriber:main',
            'hololens_subscriber = unity_robotics_demo.hololens_subscriber:main',
            'velocity_subscriber = unity_robotics_demo.velocity_subscriber:main',
            'hello_subscriber = unity_robotics_demo.hello_subscriber:main',
            #'spot_get_image = unity_robotics_demo.spot_get_image:main',
            'debug_tagandcamera_publisher = unity_robotics_demo.debug_tagandcamera_publisher:main',
            'debug_joints_publisher = unity_robotics_demo.debug_joints_publisher:main',
            'debug_velocity_publisher = unity_robotics_demo.debug_velocity_publisher:main',
            'debug_transform_publisher = unity_robotics_demo.debug_transform_publisher:main',
            'debug_hello_publisher = unity_robotics_demo.debug_hello_publisher:main'
        ],
    },
)
