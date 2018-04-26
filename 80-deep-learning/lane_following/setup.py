from setuptools import find_packages
from setuptools import setup

package_name='lane_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=['lane_following'],
    package_dir={'lane_following': 'src'},
    package_data={'lane_following': ['host/model/**/*']},
    install_requires=['setuptools', 'numpy', 'cv_bridge', 'cv2'],
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
            'dl_lane_following_node_ncs = lane_following.dl_lane_following_ncs:main'
        ],
    },
)
