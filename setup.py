#! /usr/bin/python

from setuptools import setup

setup(name='omron_dt6',
      version='0.2',
      description='Python code to configure and read the Omron DT6 thermal sensor',
      url='http://github.com/griffegg/omron_dt6',
      author='Greg Griffes',
      author_email='greg@yottametric.com',
      license='GNU GPL 3.0',
      packages=['omron_src'],
      zip_safe=False)
