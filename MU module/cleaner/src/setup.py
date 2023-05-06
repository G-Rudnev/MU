#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

setup(
    name="Python cleaner module",
    version="0.0.1",
    author="Alex",
    description="Python module to communicate with cleaner main_board",
    python_requires=">=2.7",
    packages=['modbus_tk'],
    py_modules=["cleaner","joy_control","bus_handler"],
)