#!/usr/bin/env python
""" ROS F9P Driver Python Setup"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_f9p_driver'],
    package_dir={'': 'src'}
)

setup(**d)
