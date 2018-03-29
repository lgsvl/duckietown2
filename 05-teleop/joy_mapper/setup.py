from setuptools import find_packages
from setuptools import setup

setup(
    name='joy_mapper',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Brian Shin',
    author_email='brian.shin@lge.com',
    maintainer='Brian Shin',
    maintainer_email='brian.shiN@lge.com',
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
            'joy_mapper = src.joy_mapper_node:main',
        ],
    },
)
