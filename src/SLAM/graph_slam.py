# venv/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import copy
import itertools

import matplotlib.pyplot as plt
import time


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
    def __init__(self, x=0, y=0, yaw=0):
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
            theta = math.atan2(dy, dx) + np.random.randn() * self.observe_variance[1, 1]
            theta = normalize(theta)
            if d < self.visible_distance:
                Z_observed.append([d, theta, i])

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
            l_x = X[0] + Z1[0] * math.cos(Z1[1])
            l_y = X[1] + Z1[0] * math.sin(Z1[1])
        where X means the to estimate state and Z is the correspondence observation landmark.
    X1, X2 is the two nodes,
    e: [x, y].T
    Z: [d, angle, id]
    """

    def __init__(self, ida, Z1, idb, Z2):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))
        self.Z1 = Z1
        self.Z2 = Z2
        self.ida = ida
        self.idb = idb
        self.P = np.diag([0.1, 0.1, np.radians(1.0)])

    def jacob(self):
        A = np.matrix([[-1.0, 0, 0],
                       [0, -1.0, 0],
                       [0, 0.0, 0]])
        B = np.matrix([[1.0, 0., 0],
                       [0, 1.0, 0],
                       [0, 0.0, 0]])
        return A, B

    @staticmethod
    def calc_pos(state, Z):
        x = state[0, 0] + Z[0, 0] * math.cos(Z[0, 1])
        y = state[1, 0] + Z[0, 0] * math.sin(Z[0, 1])
        return x, y

    @staticmethod
    def calc_rotation_matrix(theta):
        r = np.matrix([[math.cos(theta), -math.sin(theta), 0],
                       [math.sin(theta), math.cos(theta), 0],
                       [0, 0, 1.0]])
        return r

    def update(self, state_1, state_2):
        x_1, y_1 = self.calc_pos(state_1, self.Z1)
        x_2, y_2 = self.calc_pos(state_2, self.Z2)
        self.e[0] = x_2 - x_1
        self.e[1] = y_2 - y_1
        self.e[2] = normalize(state_2[2, 0] - state_1[2, 0])
        r1 = self.calc_rotation_matrix(self.Z1[0, 1])
        r2 = self.calc_rotation_matrix(self.Z2[0, 1])
        self.omega = np.linalg.inv(r1 * self.P * r1.T + r2 * self.P * r2.T)
        return self.e.T * self.omega * self.e


def build_graph(zlist):
    edges = dict()
    zids = list(itertools.combinations(range(len(zlist)), 2))
    for (t1, t2) in zids:
        if zlist[t1] is None or zlist[t2] is None:
            continue
        z_t1 = zlist[t1]
        z_t2 = zlist[t2]
        for iz1 in range(z_t1.shape[0]):
            for iz2 in range(z_t2.shape[0]):
                if z_t1[iz1, 2] == z_t2[iz2, 2]:
                    # matched
                    edge = Edge(t1, z_t1[iz1, :], t2, z_t2[iz2, :])
                    # edges[t1][t2].setdefault(z_t1[iz1, 2], edge)
                    edges.setdefault(hash_index(t1, t2, len(zlist), z_t1[iz1, 2], 10),
                                     edge)  # assume that there is no more than 10 landmarks.
    return edges


def hash_index(t1, t2, l, zid, zl):
    return ((t1 * l) + t2) * zl + zid


def fill_H_and_b(H, b, edge):
    """

    :param H:
    :param b:
    :param edge: Edge
    :type edge: Edge
    :return:
    """
    A, B = edge.jacob()
    id1 = edge.ida * 3
    id2 = edge.idb * 3
    H[id1: id1 + 3, id1: id1 + 3] += A.T * edge.omega * A
    H[id1: id1 + 3, id2: id2 + 3] += A.T * edge.omega * B
    H[id2: id2 + 3, id1: id1 + 3] += B.T * edge.omega * A
    H[id2: id2 + 3, id2: id2 + 3] += B.T * edge.omega * B

    b[id1: id1 + 3, 0] += (A.T * edge.omega * edge.e)
    b[id2: id2 + 3, 0] += (B.T * edge.omega * edge.e)
    return H, b


def update_edges(edges, x_opt):
    x_ids = itertools.combinations(range(x_opt.shape[1]), 2)
    for (t1, t2) in x_ids:
        for i in range(10):
            label = hash_index(t1, t2, len(x_opt), i, 10)
            if edges.has_key(label):
                edges[label].update(x_opt[:, t1], x_opt[:, t2])
    return edges


def graph_based_slam(XList, ZList, max_iter=150):
    zlist = copy.deepcopy(ZList)
    x_opt = copy.deepcopy(XList)
    nx = x_opt.shape[1]
    n = nx * x_opt.shape[0]
    edges = build_graph(zlist)

    for iter in range(max_iter):
        H = np.matrix(np.zeros((n, n)))
        b = np.matrix(np.zeros((n, 1)))
        update_edges(edges, x_opt)
        for edge in edges.values():
            H, b = fill_H_and_b(H, b, edge)
        H[0:3, 0:3] = np.identity(3)
        # H[:, :] += np.diag(([1] * n))
        dx = - np.linalg.inv(H).dot(b)
        for i in range(nx):
            x_opt[0:3, i] += dx[i * 3: i * 3 + 3, 0]

        diff = dx.T.dot(dx)
        print("iteration: %d, diff: %f" % (iter + 1, diff))
        if diff < 1.0e-5:
            break
    return x_opt


def estimate(state, u, dt):
    F = np.matrix([[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]])
    B = np.matrix([[math.cos(state[2, 0]) * dt, 0.],
                   [math.sin(state[2, 0]) * dt, 0.],
                   [0, dt]])
    return F * state + B * u


def main():
    room = Room()
    room.add_landmark(10.0, -2.0)
    room.add_landmark(15.0, 10.0)
    room.add_landmark(3.0, 15.0)
    room.add_landmark(-5.0, 20.0)
    room.add_landmark(-5.0, 5.0)

    car = Car()

    show_graph_time = 5
    hxDR = np.matrix(np.zeros((3, 0)))
    hxTrue = np.matrix(np.zeros((3, 0)))
    hz = []
    dtime = 0
    step = 0
    prev_state = car.get_state()

    while step < 80:
        step += 0.5
        dtime += 0.5

        car.move()
        u = car.get_input()

        hxTrue = np.hstack((hxTrue, car.get_state()))
        predict = estimate(prev_state, u, 0.5)
        hxDR = np.hstack((hxDR, predict))
        z = car.observe_room(room)

        hz.append(z)

        if dtime >= show_graph_time:
            x_opt = graph_based_slam(hxDR, hz)
            dtime = 0

            plt.cla()
            landmarks = room.get_landmarks_as_matrix()
            plt.plot(np.array(landmarks[:, 0]).flatten(), np.array(landmarks[:, 0]).flatten(), "*k")
            plt.plot(np.array(hxTrue[0, :]).flatten(), np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(), np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(x_opt[0, :]).flatten(), np.array(x_opt[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time" + str(step)[0:5])
            plt.pause(1.0)


if __name__ == '__main__':
    main()
