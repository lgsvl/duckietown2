from setuptools import find_packages
from setuptools import setup

package_name='duckietown'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    #setup_requires=['numpy'],
    install_requires=['setuptools', 'numpy', 'python-opencv'],
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
    description='Port of original duckietown package to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
        ],
    },
)
