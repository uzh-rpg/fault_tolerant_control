#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ze_imu'],
    package_dir={'': 'py'},
    install_requires=['rospy', 'yaml'],
    )

setup(**d)