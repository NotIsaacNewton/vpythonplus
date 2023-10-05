#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 17:45:27 2021

@author: ariandovald
"""
from vpythonplus import *

universe = Universe(solver="verlet", dt=0.000005, sound=False)

body1 = Sphere(universe, mass=12*10**12, radius=0.1, pos=vec(0, 0, 0),
               vel=vec(-14, 14, 0), make_trail=True, trail_radius=0.01)
body2 = Sphere(universe, mass=12*10**12, radius=0.1, pos=vec(1, 0, 0),
               vel=vec(0, -28, 0), make_trail=True, trail_radius=0.01)

universe.start(time=0.000)
