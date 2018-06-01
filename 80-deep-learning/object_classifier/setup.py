from setuptools import find_packages
from setuptools import setup

package_name='object_classifier'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    # package_dir={'lane_following': 'src'},
    #              'lane_following': 'launch'},
    # package_data={'object_classifier': ['host/model/**/*']},
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
    description='Image recognition node using Inception-v4',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'object_classification_node = object_classifier.object_classification:main'
        ],
    },
)
