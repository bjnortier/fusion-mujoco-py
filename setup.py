# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='fusion-mujoco-py',
    version='0.1.0',
    description='Export Autodesk Fusion to Mujoco XML model',
    long_description=readme,
    author='Ben Nortier',
    author_email='ben@bjnortier.com',
    url='https://github.com/bjnortier/fusion-mujoco-py',
    license=license,
    packages=find_packages(exclude=('tests', 'docs'))
)
