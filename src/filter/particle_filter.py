# env/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.stats
import random

class Car(object):
    SENSOR_RANGE = 1000
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.yawrate = 0
        self.a = 0
        self.process_variance = np.matrix(np.diag([0.1, np.radians(10)]))
        self.observe_variance = np.matrix(np.diag([0.1, 0.1]))

    def move(self, yawrate=1, a=1, dt=0.1):
        self.x = self.x + self.v * dt * math.cos(self.yaw)
        self.y = self.y + self.v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + dt * yawrate
        self.v = self.v + dt * a
        self.yawrate = yawrate
        self.a = a

    def get_input(self):
        a = self.a + np.random.randn() * self.process_variance[0,0]
        yawrate = self.yawrate + np.random.randn() * self.process_variance[1, 1]
        return np.matrix([a, yawrate]).T

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

    def state(self):
        return np.matrix([self.x, self.y, self.yaw, self.v]).T


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
        self.weights = np.matrix(np.zeros((number_particles, 1))) + 1.0 / number_particles
        self.NTH = number_particles * 1 / 4
        print(type(self.weights))

    def _initial_prarticles(self, initial_X, initial_std, number_particles):
        particles = np.matrix(np.zeros((initial_X.shape[0], number_particles)))
        particles[0, :] = initial_X[0,0] + (np.random.randn(number_particles)) * initial_std[0, 0]
        particles[1, :] = initial_X[1,0] + (np.random.randn(number_particles)) * initial_std[1, 0]
        particles[2, :] = initial_X[2, 0] + (np.random.randn(number_particles)) * initial_std[2, 0]
        particles[2, :] %= (2 * np.pi)
        particles[3, :] = initial_X[3, 0] + (np.random.randn(number_particles)) * initial_std[3, 0]
        return particles

    def _move_motion(self, X, u, dt):
        F = np.matrix([[1., 0., 0., dt * math.cos(X[2])],
                       [0., 1., 0., dt * math.sin(X[2])],
                       [0., 0., 1., 0.],
                       [0., 0., 0., 1.]])
        B = np.matrix([[0, 0.],
                       [0, 0.],
                       [0., dt],
                       [dt, 0.]])
        X_pred = F * X + B * u
        return X_pred

    def calc_convariance(self, X_mean):
        covariance = np.zeros((4, 4))
        for i in range(self.np):
            dx = self.particles[:, i:i+1] - X_mean
            covariance += self.weights[i, 0] * dx * dx.T
        return covariance

    def predict(self, u, Q, dt=0.1):
        for i in range(self.np):
            u_p = np.zeros((2, 1))
            u_p[0, 0] = u[0,0] + np.random.randn() * Q[0, 0]
            u_p[1, 0] = u[1,0] + np.random.randn() * Q[1, 1]
            self.particles[:, i:i+1] = self._move_motion(self.particles[:, i:i+1], u_p, dt)
        X = self.particles * self.weights
        P = self.calc_convariance(X)
        return X, P

    def gauss_likelihood(self, x, sigma):
        p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * math.exp(-x **2/(2 * sigma ** 2))
        return p

    def update(self, Z_observed, R):
        for i in range(self.np):
            X = self.particles[:, i]
            # w = self.weights[i, 0]
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
        X = self.particles * self.weights
        P = self.calc_convariance(X)
        return X, P

    def random(self):
        resampleid = np.zeros((1, self.np))
        for i in range(self.np):
            resampleid[0, i] = random.uniform(0, 1.0)
        # base = np.cumsum(self.weights * 0.0 + 1 / self.np) - 1 / self.np
        # resampleid = base + np.random.randn(base.shape[1]) / self.np
        return resampleid

    def resample(self):
        self.weights += 1.e-300
        Neff = 1.0 / (self.weights.T * self.weights)[0, 0]
        if Neff < self.NTH:
            wcum = np.cumsum(self.weights, axis=0)
            wcum[-1, 0] = 1.
            indexes = []
            resampleid = self.random()
            for i in range(self.np):
                ind = 0
                while wcum[ind, 0] < resampleid[0, i]:
                    ind += 1
                indexes.append(ind)

            self.particles[:, :] = self.particles[:, indexes]
            self.weights[:, 0] = self.weights[indexes, 0]
            self.weights /= np.sum(self.weights)

    def get_particles(self):
        return self.particles


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    #eigval[bigind] or eiqval[smallind] were occassionally negative numbers extremely
    #close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try: a = math.sqrt(eigval[bigind])
    except ValueError: a = 0

    try: b = math.sqrt(eigval[smallind])
    except ValueError: b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.matrix([[math.cos(angle), math.sin(angle)],
                   [-math.sin(angle), math.cos(angle)]])
    fx = R * np.matrix([x, y])
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    room = Room()
    room.add_landmark(10.0, 0)
    room.add_landmark(10.0, 10.0)
    room.add_landmark(0.0, 15.0)
    room.add_landmark(-5.0, 20.0)

    robot = Car()
    initial_X = np.matrix([0.0, 0.0, 0.0, 0.0]).T
    initial_std = np.matrix([0.1, 0.1, 0.1, 0.1]).T
    particles = ParticleFilter(initial_X, initial_std, 100)

    Q = np.diag([0.1, np.radians(5)]) # process variance
    R = np.diag([0.5, 0.5]) # measurement noise variance

    sim_time = 0

    hxEst = initial_X
    hxTrue = initial_X
    PEst = np.eye(4)
    dt = 0.1

    while sim_time < 100:
        sim_time += dt
        robot.move()
        u = robot.get_input()

        X_pred, P_pred = particles.predict(u, Q, dt)

        z_observed = robot.observe(room)
        X_est, P_est = particles.update(z_observed, R)

        hxEst = np.hstack((hxEst, X_est))
        X_true = robot.state()
        hxTrue = np.hstack((hxTrue, X_true))

        plt.cla()

        # for i in range(len(z_observed)):
        #     plt.plot([X_true[0, 0], z_observed[i][1]], [X_true[1, 0], z_observed[i][2]], "-k")

        for landmark in room.get_landmarks():
            plt.plot(landmark[0], landmark[1], "*k")

        plt.plot(particles.particles[0, :], particles.particles[1, :], ".r")

        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b")

        plt.plot(np.array(hxEst[0, :]).flatten(),
                 np.array(hxEst[1, :]).flatten(), "-y")

        plot_covariance_ellipse(X_est, P_est)

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

    plt.pause(1000)
if __name__ == '__main__':
    main()
