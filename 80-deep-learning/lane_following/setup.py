from setuptools import find_packages
from setuptools import setup

package_name='lane_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    #package_dir={'lane_following': 'src'},
    #             'lane_following': 'launch'},
    package_data={'lane_following': ['host/model/**/*']},
    install_requires=['launch', 'setuptools'],
    author='David Uhm',
    author_email='david.uhm@lge.com',
    maintainer='David Uhm',
    maintainer_email='david.uhm@lge.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Port of End-To-End Deep Neural network lane following algorithm to ROS2',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'dl_lane_following_node = lane_following.dl_lane_following:main',
            'dl_lane_following_node_ncs = lane_following.dl_lane_following_ncs:main',
            'dl_lane_following-demo = lane_following_launch.dl_lane_following_launch:main',
            'dl_lane_following_ncs-demo = lane_following_launch.dl_lane_following_ncs_launch:main'
        ],
    },
)
