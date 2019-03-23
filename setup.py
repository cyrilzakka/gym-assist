#!/usr/bin/env python
from setuptools import setup

setup(name='gym_assist',
      version='0.0.1',
      install_requires=['gym>=0.12.0', 
      					'mujoco-py<2.1,>=2.0.2.0a1', 
      					'imageio']
     )  