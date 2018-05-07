from setuptools import find_packages
from setuptools import setup

package_name='dagu_car'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    #package_dir={'': 'dagu_car/dagu_car_include'},
    setup_requires=['numpy'],
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
    description='Port of duckietown dagu_car package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'inverse_kinematics_node = dagu_car.inverse_kinematics_node:main',
            'wheels_driver_node = dagu_car.wheels_driver_node:main',
            'car_cmd_switch_node = dagu_car.car_cmd_switch_node:main'
        ],
    },
)
