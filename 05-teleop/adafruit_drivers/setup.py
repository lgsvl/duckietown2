from setuptools import find_packages
from setuptools import setup

package_name='adafruit_drivers'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('include', exclude=['test']),
    #packages=['Adafruit_ADS1x15', 'Adafruit_GPIO','Adafruit_I2C','Adafruit_LSM303','Adafruit_MotorHAT','Adafruit_PWM_Servo_Driver','Gyro_L3GD20'],
    package_dir={'': 'include'},
    setup_requires=[],
    install_requires=['setuptools'],
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
    description='Port of duckietown adafruit_drivers package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
        ],
    },
)
