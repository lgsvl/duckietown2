from setuptools import find_packages
from setuptools import setup

package_name = 'pi_camera'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    keywords=['ROS'],
    description='Port of duckietown pi_camera package to ROS2',
    entry_points={
        'console_scripts': [
            'camera_node_sequence = pi_camera.camera_node_sequence:main'
        ],
    },
)
