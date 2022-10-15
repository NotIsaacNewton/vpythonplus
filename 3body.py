#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 17:45:27 2021

@author: ariandovald
"""
from vpython import *
from vpythonplus import Sphere

planet1 = Sphere(pos=vec(0, 0, 0), mass=1000, radius=0.1, make_trail=True)
planet2 = Sphere(pos=vec(1, 0, 0), mass=1000, radius=0.1, vel=vec(0, 0.00025, 0), make_trail=True)
planet3 = Sphere(pos=vec(0, -1, 0), mass=1000, radius=0.1, vel=vec(0, 0, 0.00025), make_trail=True)

while planet1.t < 1000000:
    planet1.gravity()
    planet2.gravity()
    planet3.gravity()
    planet1.move()
    planet2.move()
    planet3.move()
