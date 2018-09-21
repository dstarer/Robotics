# venv/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import copy
import itertools

import matplotlib.pyplot as plt


def normalize(angle):
    while angle > np.pi * 2:
        angle -= np.pi
    while angle < - np.pi * 2:
        angle += np.pi
    return angle


class Room(object):
    def __init__(self):
        self.landmarks = []

    def add_landmark(self, x, y):
        return self.landmarks.append([x, y])

    def get_landmarks_as_matrix(self):
        return np.matrix(self.landmarks)

    def number(self):
        return len(self.landmarks)


class Car(object):
    def __init__(self, x = 0, y = 0, yaw = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0
        self.delta = 0
        self.process_variance = np.matrix(np.diag([0.5, np.radians(10)]))
        self.observe_variance = np.matrix(np.diag([0.5, np.radians(10)]))
        self.visible_distance = 20

    def move(self, v=2, delta=np.radians(10), dt=0.1):
        self.x = self.x + v * dt * math.cos(self.yaw)
        self.y = self.y + v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + delta
        self.v = v + np.random.randn() * self.process_variance[0, 0]
        self.delta = delta + np.random.randn() * self.process_variance[1, 1]

    def get_input(self):
        return np.matrix([self.v, self.delta]).T

    def observe_room(self, room):
        """

        :param room:
        :type room: Room
        :return:
        """
        Z_observed = []
        for i, lm in enumerate(room.landmarks):
            dx = lm[0] - self.x
            dy = lm[1] - self.y
            d = math.sqrt(dx * dx + dy * dy) + np.random.randn() * self.observe_variance[0, 0]
            theta = math.atan2(dy, dx) - self.yaw + np.random.randn() * self.observe_variance[1, 1]
            phi = math.atan2(dy, dx)
            normalize(theta)
            if d < self.visible_distance:
                Z_observed.append([d, theta, phi, i])

        return np.matrix(Z_observed)

    def get_state(self):
        return np.matrix([self.x, self.y, self.yaw]).T


class Edge(object):
    """
    The robot observed the same landmark in two different positions X1 and X2.
    Our mission is to correct the X1, X2 so that minimize the error H(X1) - H(X2).
    f([X1, X2].T) = H(X1) - H(X2)
    cost function: F([X1, X2].T) = f([X1, X2].T).T * Omega * f([X1, X2].T), with Omega measures the data's noise.
    f([X1, X2].T + [delta_X1, delta_x2].T) = f([X1, X2].T) + J([X1, X2].T) * [delta_x1, delta.X2].T
    to minimize F([X1, X2].T):
        expand F([X1, X2].T)
        and derive
            J([X1, X2].T).T * Omega * J([X1, X2].T) * [delta_X1, delta_X2].T= - J([X1, X2].T).T * Omega * f([X1, X2].T)
            so get the [delta_x1, delta_x2],
        J([X1, X2].T) = f'([X1, X2].T) = (H(X1) - H(X2))' = [H'(X1), - H'(X2)].T
        H(X):
            l_x = X[0] + Z1[0] * math.cos(X[2] + Z1[1])
            l_y = X[1] + Z1[0] * math.sin(X[2] + Z1[1])
            l_theta = X[2] + Z1[1] + Z1[2]
        where X means the to estimate state and Z is the correspondence observation landmark.
    X1, X2 is the two nodes,
    e: [x, y, theta].T
    Z: [d, angle, phi, id]
    """
    def __init__(self):
        self.e = np.zeros((3, 1))
        self.Omega = np.zeros((3, 3))
        self.Z1 = np.zeros((4, 1))
        self.Z2 = np.zeros((4, 1))

    def jacobian(self):
        t1 = self.Z1[1] + self.Z1[2]