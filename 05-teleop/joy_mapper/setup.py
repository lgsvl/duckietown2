from setuptools import find_packages
from setuptools import setup

package_name = 'joy_mapper'

setup(
    name=package_name,
    version='1.0.1',
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
    description='Port of duckietown joy_mapper package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'joy_mapper_node = joy_mapper.joy_mapper_node:main',
            'joystick_launch = joy_mapper_launch.joystick_launch:main'
        ],
    },
)
