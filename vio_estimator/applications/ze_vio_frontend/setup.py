#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ze_vio_frontend'],
    package_dir={'': 'py'},
    install_requires=['yaml'],
    )

setup(**d)
