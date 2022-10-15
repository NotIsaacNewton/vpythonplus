#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 17:45:27 2021

@author: ariandovald
"""
from vpython import *


class Sphere(sphere):
    vel = vec(0, 0, 0)
    acc = vec(0, 0, 0)
    fnet = vec(0, 0, 0)
    mass = 0
    charge = 0
    t = 0
    dt = 0.01
    objects = []

    def __init__(self, **args):
        super().__init__(**args)
        Sphere.objects.append(self)

    def move(self):
        self.acc = self.fnet / self.mass
        self.vel += self.acc * self.dt
        self.pos += self.vel * self.dt
        self.t += self.dt
        self.fnet = vec(0, 0, 0)

    def gravity(self):
        for o in Sphere.objects:
            if o != self:
                self.fnet += 6.7e-11 * o.mass * self.mass * norm(o.pos - self.pos) / (mag(self.pos - o.pos) ** 2)

    def electric(self):
        for o in Sphere.objects:
            if o != self:
                self.fnet += 9e9 * o.charge * self.charge * norm(o.pos - self.pos) / (mag(self.pos - o.pos) ** 2)
