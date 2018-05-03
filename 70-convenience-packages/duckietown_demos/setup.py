from setuptools import find_packages
from setuptools import setup

package_name = 'duckietown_demos'

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
    description='Port of duckietown_demos package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'joystick-demo = duckietown_demos_launch.joystick_launch:main',
            'lane-following-demo = duckietown_demos_launch.lane_following_launch:main',
            'lane-following-demo-sim = duckietown_demos_launch.lane_following_simulator_launch:main'
        ],
    },
)
