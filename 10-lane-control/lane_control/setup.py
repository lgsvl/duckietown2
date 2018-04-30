from setuptools import find_packages
from setuptools import setup

package_name='lane_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    #package_dir={'': 'dagu_car/dagu_car_include'},
    #setup_requires=['numpy'],
    install_requires=['setuptools', 'numpy'],
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
    description='Port of original duckietown lane control package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'lane_controller_node = lane_control.lane_controller_node:main',
            'lane_following_launch = lane_control.lane_control_launch.lane_following_launch:main'
        ],
    },
)
