# env/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.stats


class Car(object):
    SENSOR_RANGE = 10
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.yawrate = 0
        self.a = 0
        self.process_variance = np.matrix(np.diag([0.1, 0.2]))
        self.observe_variance = np.matrix(np.diag([0.1, 0.1]))

    def move(self, yawrate=1, a=1, dt=0.1):
        self.x = self.x + self.v * dt * math.cos(self.yaw)
        self.y = self.y + self.v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + dt * yawrate
        self.v = self.v + a * dt
        self.yawrate = yawrate
        self.a = a

    def get_input(self):
        a = self.a + np.random.randn() * self.process_variance[0,0]
        yawrate = self.yawrate + np.random.randn() * self.process_variance[1, 1]
        return a, yawrate

    def observe(self, room):
        """
        :param room:
        :type room: Room
        :return:
        """
        sensor_data = []
        for lx, ly in room.get_landmarks():
            dist = math.sqrt((self.x - lx) * (self.x - lx) + (self.y - ly) * (self.y - ly))
            if dist < self.SENSOR_RANGE:
                dist_with_noise = dist + np.random.randn() * self.observe_variance[0, 0]
                sensor_data.append([dist_with_noise, lx, ly])
        return sensor_data


class Room(object):
    def __init__(self):
        self.landmarks = []

    def add_landmark(self, x, y):
        self.landmarks.append([x, y])

    def get_landmarks(self):
        return self.landmarks

    def get_number_landmarks(self):
        return len(self.landmarks)


class ParticleFilter(object):
    def __init__(self, initial_X, initial_std, number_particles):
        self.np = number_particles
        self.particles = self._initial_prarticles(initial_X, initial_std, number_particles)
        self.weights = np.zeros((number_particles, 1)) + 1.0 / number_particles
        self.NTH = number_particles / 2

    def _initial_prarticles(self, initial_X, initial_std, number_particles):
        particles = np.empty((initial_X.shape[1], number_particles))
        particles[0, :] = initial_X[0,0] + (np.random.randn(number_particles)) * initial_std[0, 0]
        particles[1, :] = initial_X[1,0] + (np.random.randn(number_particles)) * initial_std[1, 0]
        particles[2, :] = initial_X[2, 0] + (np.random.randn(number_particles)) * initial_std[2, 0]
        particles[2, :] %= (2 * np.pi)
        particles[3, :] = initial_X[3, 0] + (np.random.randn(number_particles)) * initial_std[3, 0]
        return particles

    def _move_motion(self, X, u, dt):
        F = np.matrix([[1., 0., 0., 0.],
                       [0., 1., 0., 0.],
                       [0., 0., 1., 0.],
                       [0., 0., 0., 1.]])
        B = np.matrix([[dt * math.cos(X[2, 0]), 0],
                       [dt * math.sin(X[2, 0]), 0],
                       [0, dt],
                       [dt, 0]])
        X_pred = F * X + B * u
        return X_pred

    def calc_convariance(self, X_mean):
        covariance = np.zeros((4, 4))
        for i in range(self.np):
            dx = self.particles[:, i] - X_mean
            covariance += self.weights[i, 0] * dx * dx.T
        return covariance

    def predict(self, u, Q, dt=0.1):
        for i in range(self.np):
            u_p = np.zeros((2, 1))
            u_p[0, 0] = u[0] + np.random.randn() * Q[0, 0]
            u_p[1, 0] = u[1] + np.random.randn() * Q[1, 1]
            self.particles[:, i] = self._move_motion(self.particles[:, i], u_p, dt)
        X = self.particles * self.weights
        P = self.calc_convariance(X)
        return X, P

    def gauss_likelihood(self, x, sigma):
        p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * math.exp(-x **2/(2 * sigma ** 2))
        return p

    def update(self, Z_observed, R):
        for i in range(self.np):
            X = self.particles[:, i]
            w = 1.0
            for z_j, lx_j, ly_j in Z_observed:
                dx = lx_j - X[0]
                dy = ly_j - X[1]
                z_pre = math.sqrt(dx * dx + dy * dy)
                dz = z_j - z_pre
                w = w * self.gauss_likelihood(dz, math.sqrt(R[0, 0]))
            self.weights[i, 0] = w

        self.weights += 1.e-300
        self.weights = self.weights / self.weights.sum()

        # try to resample
        self.resample()

    def random(self, n):
        temp = np.array([1/n] * n)
        base = np.cumsum(temp) - 1 / n
        resampleid = base + np.random.rand(base.shape[1]) / n
        return resampleid

    def resample(self):
        Neff = 1.0 / (self.weights.T * self.weights)[0, 0]
        if Neff < self.NTH:
            wcum = np.cumsum(self.weights, axis=0)
            wcum[-1, 0] = 1.
            indexes = np.searchsorted(wcum, self.random(self.np))

            self.particles[:, :] = self.particles[:, indexes]
            self.weights[:, 0] = self.weights[indexes, 0]
            self.weights /= np.sum(self.weights)
