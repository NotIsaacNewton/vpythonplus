#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 17:45:27 2021

@author: ariandovald
"""
import threading
import queue
import math
from vpython import *
from scamp import *
from scamp_extensions.pitch import *
import numpy as np


class Universe:
    def __init__(self, solver="", magnetic=vec(0, 0, 0), electric=vec(0, 0, 0), gravitational=vec(0, 0, 0), dt=0.01,
                 forces=["electric", "magnetic", "gravity"], sound=False):
        # solver to use (either euler or verlet)
        self.solver = solver

        # physical values of environment
        # magnetic field
        self.B = magnetic
        # electric field
        self.E = electric
        # gravitational force
        self.G = gravitational

        # active forces in universe
        self.forces = forces

        # universal list of objects
        self.objects = []

        # universal list of object pairs
        self.pairs = []

        # universal list of physics objects
        self.physics = []

        # universal clock
        self.t = 0
        self.dt = dt

        # tracks initial state of verlet
        self.n = 0

        self.sound = sound

        if self.sound:
            self.s = Session(max_threads=4000)
            # self.s.print_default_soundfont_presets()
            self.instrument = self.s.new_part("cello")

    # adds objects to universal list
    def add_object(self, o):
        self.objects.append(o)

    # adds physics objects to universal list
    def add_physics_object(self, o):
        self.physics.append(o)

    # euler solver
    def euler(self):
        self.force()
        for o in self.objects:
            if o in self.physics:
                o.acc = o.fnet / o.mass
            o.vel += o.acc * self.dt
            o.posn += o.vel * self.dt
            o.fnet = vec(0, 0, 0)
        self.t += self.dt
        for o in self.objects:
            o.pos = vec(o.posn)

    # verlet solver
    def verlet(self):
        self.force()
        match self.n:
            case 0:
                for o in self.objects:
                    if o in self.physics:
                        o.acc = o.fnet / o.mass
                    o.pos0 = vec(o.posn)
                    o.posn += (o.vel * self.dt) + (o.acc * (self.dt ** 2) / 2)
                    o.fnet = vec(0, 0, 0)
                for o in self.objects:
                    o.pos = vec(o.posn)
                self.n = 1
            case 1:
                for o in self.objects:
                    if o in self.physics:
                        o.acc = o.fnet / o.mass
                    o.vel = o.posn - o.pos0
                    o.pos0 = vec(o.posn)
                    o.posn += o.vel + (o.acc * (self.dt ** 2))
                    o.fnet = vec(0, 0, 0)
                for o in self.objects:
                    o.pos = vec(o.posn)
        self.t += self.dt

    # rk4 solver
    def rk4(self):
        for o in self.physics:
            # k1
            self.force()
            k1 = vec((o.fnet * self.dt / o.mass) + o.vel)
            o.fnet = vec(0, 0, 0)
            # k2
            self.t += self.dt / 2
            o.posn = o.pos + (k1 * self.dt / 2)
            self.force()
            k2 = vec((o.fnet * self.dt / o.mass) + o.vel)
            o.fnet = vec(0, 0, 0)
            # k3
            o.posn = o.pos + (k2 * self.dt / 2)
            self.force()
            k3 = vec((o.fnet * self.dt / o.mass) + o.vel)
            o.fnet = vec(0, 0, 0)
            # k4
            self.t += self.dt / 2
            o.posn = o.pos + (k3 * self.dt)
            self.force()
            k4 = vec((o.fnet * self.dt / o.mass) + o.vel)
            o.fnet = vec(0, 0, 0)
            # update position and time
            o.posn = vec(o.pos)
            o.posn += (1 / 6) * (k1 + (2 * k2) + (2 * k3) + k4) * self.dt
            print("")
            self.t -= self.dt
        for o in self.objects:
            o.pos = vec(o.posn)
        self.t += self.dt

    # updates the universe by one step "dt" in time
    def update(self):
        match self.solver:
            case "":
                print("Please specify solver (euler or verlet).")
            # euler method
            case "euler":
                self.euler()
            # verlet method
            case "verlet":
                self.verlet()
            # classic runge-kutta (RK4) method
            case "rk4":
                self.rk4()

    def sing(self, note):
        for (o,p) in self.pairs:
                if p != o:
                    max_harmonic = 59
                    k = 0.1
                    x0 = 0

                    distance = mag(o.pos - p.pos)
                    harmonic_number = round(((1 + np.tanh(k * (distance - x0)))/2) * max_harmonic) -29

                    fundamental_frequency = 1050
                    frequency = hertz_to_midi(fundamental_frequency / (2 * harmonic_number))

                    if frequency != o.frequency0 and frequency != p.frequency0:
                        o.frequency0 = frequency
                        p.frequency0 = frequency
                        self.instrument.play_note(frequency, 1.0, self.dt, blocking=False)

    def update_thread(self, time):
        if time == 0:
            while True:
                self.update()
        else:
            while self.t < time:
                self.update()

    def sing_thread(self, time):
        note = self.instrument.start_note(0, 0.5)
        if time == 0:
            while True:
                self.sing(note)
        else:
            while self.t < time:
                self.sing(note)

    # starts the simulation
    def start(self, time=0):
        for o in self.objects:
            for p in self.objects:
                if p!= o:
                    if (p,o) not in self.pairs:
                        self.pairs.append((o,p))

        update_thread = threading.Thread(target=self.update_thread, args=(time,))
        if self.sound:
            sing_thread = threading.Thread(target=self.sing_thread, args=(time,))

        update_thread.start()
        if self.sound:
            sing_thread.start()

        update_thread.join()
        if self.sound:
            sing_thread.join()

    # calculates all forces
    def force(self):
        for f in self.forces:
            match f:
                case "electric":
                    self.electric()
                case "magnetic":
                    self.magnetic()
                case "gravity":
                    self.gravity()

    # calculates gravitational force
    def gravity(self):
        for o in self.physics:
            o.fnet += self.G
        for (o,p) in self.pairs:
            if p != o:
                o.fnet += 6.7e-11 * p.mass * o.mass * norm(p.pos - o.posn) / (mag(o.posn - p.pos) ** 2)
                p.fnet -= 6.7e-11 * p.mass * o.mass * norm(p.pos - o.posn) / (mag(o.posn - p.pos) ** 2)

    # calculates electric force
    def electric(self):
        for o in self.physics:
            o.fnet += o.charge * self.E
            for p in self.objects:
                if p != o:
                    o.fnet += 9e9 * p.charge * o.charge * norm(o.posn - p.pos) / (mag(o.posn - p.pos) ** 2)

    # calculates magnetic force
    def magnetic(self):
        for o in self.physics:
            o.fnet += o.charge * cross(o.vel, self.B)
            for p in self.objects:
                if p != o:
                    o.fnet += (10 ** (-7)) * o.charge * p.charge * o.vel.cross(p.vel.cross(norm(o.posn - p.pos))) / \
                              (mag(o.posn - p.pos) ** 2)


class Sphere(sphere):
    def __init__(self, universe, physics=True, vel=vec(0, 0, 0), acc=vec(0, 0, 0), fnet=vec(0, 0, 0),
                 mass=0, charge=0, **args):
        super().__init__(**args)
        # add self into universe list
        universe.add_object(self)
        # is the object affected by physics
        if physics:
            universe.add_physics_object(self)
        # physical values of object
        self.vel = vel
        self.acc = acc
        self.fnet = fnet
        self.pos0 = vec(0, 0, 0)
        self.mass = mass
        self.charge = charge
        # for runge-kutta method
        self.posn = vec(self.pos)

        self.frequency0 = 0
