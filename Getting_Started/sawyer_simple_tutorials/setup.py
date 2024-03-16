#! /usr/bin/env python
# from setuptools import setup
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sawyer_simple_tutorials'],
    scripts=['scripts'],
)

setup(**d)
