from robot import robot
from math import pi
from copy import deepcopy
import random

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class Particles:

    def __init__(self, x, y, heading,
                 turning_noise, distance_noise, measurement_noise, N = 100):
        self.N = N
        self.turning_noise = turning_noise
        self.distance_noise = distance_noise
        self.measurement_noise = measurement_noise

        self.data = []
        for i in range(self.N):
            r = robot(x, y, heading)
            r.set_noise(turning_noise, distance_noise, measurement_noise)
            self.data.append(r)

    def get_position(self):
        x = 0.0
        y = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y

        return (x/self.N, y/self.N)

    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):

        for i in range(self.N):
            self.data[i].move(turning, distance, tolerance, max_turning_angle)

    def sense(self, measurement):
        w = []
        for i in range(self.N):
            w.append(self.data[i].compute_weight(measurement))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(deepcopy(self.data[index]))
        self.data = p3
