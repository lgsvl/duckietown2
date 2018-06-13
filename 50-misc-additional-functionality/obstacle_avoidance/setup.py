from setuptools import find_packages
from setuptools import setup

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['launch', 'setuptools'],
    author='Brian Shin',
    author_email='brian.shin@lge.com',
    maintainer='Brian Shin',
    maintainer_email='brian.shin@lge.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 obstacle avoidance package for Duckiebot',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'range_sensors_node = obstacle_avoidance.range_sensors_node:main',
            'range_sensors_cmd_switch_node = obstacle_avoidance.range_sensors_cmd_switch_node:main'
        ],
    },
)
