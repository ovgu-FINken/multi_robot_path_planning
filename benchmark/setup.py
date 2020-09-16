#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      December 2019
@brief:     Setup.py to find py-files in other packages.
@todo:
------------------------------------------------------------- """


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['benchmark'],
    package_dir={'': 'src'},
)

setup(**setup_args)
