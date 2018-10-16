# venv/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import copy
import itertools

import matplotlib.pyplot as plt
import time


def normalize(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Room(object):
    """
    one Room contains several landmarks,
    each landmark has its' location (x, y) and orientation.
    """

    def __init__(self):
        self.landmarks = []

    def add_landmark(self, x, y, alpha):
        return self.landmarks.append([x, y, alpha])

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
        self.process_variance = np.matrix(np.diag([0.2, np.radians(5.0)]))
        self.observe_variance = np.matrix(np.diag([0.1, np.radians(10.0), np.radians(5)]))
        self.visible_distance = 20

    def move(self, v=1, delta=0.1, dt=0.1):
        self.x = self.x + v * dt * math.cos(self.yaw)
        self.y = self.y + v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + delta * dt

        self.v = v + np.random.randn() * self.process_variance[0, 0]
        self.delta = delta + np.random.randn() * self.process_variance[1, 1]

    def get_input(self):
        return np.matrix([self.v, self.delta]).T

    def observe_room(self, room):
        """

        :param room:
        :type room: Room
        :return: observed data.
        each observed data is a triple (d_t, phi_t, beta_t)
        """
        Z_observed = []
        for i, lm in enumerate(room.landmarks):
            dx = lm[0] - self.x
            dy = lm[1] - self.y
            d = math.sqrt(dx * dx + dy * dy)
            phi_t = (math.atan2(dy, dx) - self.yaw)
            beta_t = lm[2] - (phi_t + self.yaw)
            theta = normalize(phi_t)
            beta_t = normalize(beta_t)
            if d < self.visible_distance:
                d = d + np.random.randn() * self.observe_variance[0, 0]
                theta = theta + np.random.randn() * self.observe_variance[1, 1]
                Z_observed.append([d, theta, beta_t, i])

        return np.matrix(Z_observed)

    def get_state(self):
        return np.matrix([self.x, self.y, self.yaw]).T


class NoVEdge(object):
    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))
        self.d1 = 0
        self.angle1 = 0
        self.phi1 = 0

        self.yaw1 = 0
        self.x1 = 0
        self.y1 = 0

        self.d2 = 0
        self.angle2 = 0
        self.phi2 = 0

        self.yaw2 = 0
        self.x2 = 0
        self.y2 = 0

        self.t1 = 0
        self.t2 = 0

    def setup(self, d1, angle1, phi1, d2, angle2, phi2, t1, t2):
        self.angle1 = angle1
        self.angle2 = angle2
        self.phi2 = phi2
        self.phi1 = phi1
        self.d1 = d1
        self.d2 = d2
        self.t1 = t1
        self.t2 = t2

    def update(self, x1, y1, yaw1, x2, y2, yaw2):
        self.x1 = x1
        self.y1 = y1
        self.yaw1 = yaw1
        self.x2 = x2
        self.y2 = y2
        self.yaw2 = yaw2

        x_t1, y_t1, alpha_t1 = self.cal_landmark_pose(x1, y1, yaw1, self.d1, self.angle1, self.phi1)
        x_t2, y_t2, alpha_t2 = self.cal_landmark_pose(x2, y2, yaw2, self.d2, self.angle2, self.phi2)
        self.e[0, 0] = x_t2 - x_t1
        self.e[1, 0] = y_t2 - y_t1
        self.e[2, 0] = normalize(alpha_t2 - alpha_t1)
        self.omega = np.eye(3)
        cost = self.e.T * self.omega * self.e
        return cost[0, 0]

    @staticmethod
    def cal_landmark_pose(x, y, yaw, d, angle, phi):
        xl = x + d * math.cos(yaw + angle)
        yl = y + d * math.sin(yaw + angle)
        alpha = yaw + angle + phi
        return xl, yl, alpha

    def jacobian(self):
        t1 = self.yaw1 + self.angle1
        t2 = self.yaw2 + self.angle2
        A = np.matrix([[-1.0, 0, self.d1 * math.sin(t1)],
                       [0, -1.0, -self.d1 * math.cos(t1)],
                       [0, 0, -1.0]])
        B = np.matrix([[1.0, 0, -self.d2 * math.sin(t2)],
                       [0, 1.0, self.d2 * math.cos(t2)],
                       [0, 0, 1.0]])
        return A, B


# class Edge(object):
#     """
#     The robot observed the same landmark in two different positions X1 and X2.
#     Our mission is to correct the X1, X2 so that minimize the error H(X1) - H(X2).
#     f([X1, X2].T) = H(X1) - H(X2)
#     e: [x, y, alpha].T
#     Z: [d, angle, beta, id]
#     """
#
#     def __init__(self, ida, Z1, idb, Z2, lm):
#         self.e = np.zeros((3, 1))
#         self.omega = np.zeros((3, 3))
#         self.Z1 = np.zeros((1, 4))
#         self.Z1[:, :] = Z1[:, :]
#         self.Z2 = np.zeros((1, 4))
#         self.Z2[:, :] = Z2[:, :]
#         self.ida = ida
#         self.idb = idb
#         self.lm = lm
#         self.P = np.diag([0.1, 0.1, np.radians(1.0)])
#         self.state_1 = np.zeros((3, 1))
#         self.state_2 = np.zeros((3, 1))
#
#     def jacob(self):
#         t1 = self.state_1[2, 0] + self.Z1[0, 1]
#         t2 = self.state_2[2, 0] + self.Z2[0, 1]
#
#         A = np.matrix([[-1.0, 0, self.Z1[0, 0] * math.sin(t1)],
#                        [0, -1.0, -self.Z1[0, 0] * math.cos(t1)],
#                        [0, 0.0, -1.0]])
#         B = np.matrix([[1.0, 0., -self.Z2[0, 0] * math.sin(t2)],
#                        [0, 1.0, self.Z2[0, 0] * math.cos(t2)],
#                        [0, 0.0, 1.0]])
#         return A, B
#
#     @staticmethod
#     def calc_landmark(state, Z):
#         x = state[0, 0] + Z[0, 0] * math.cos(Z[0, 1] + state[2, 0])
#         y = state[1, 0] + Z[0, 0] * math.sin(Z[0, 1] + state[2, 0])
#         alpha = Z[0, 2] + Z[0, 1] + state[2, 0]
#         return x, y, alpha
#
#     @staticmethod
#     def calc_rotation_matrix(theta):
#         r = np.matrix([[math.cos(theta), -math.sin(theta), 0],
#                        [math.sin(theta), math.cos(theta), 0],
#                        [0, 0.0, 1.0]])
#         return r
#
#     def update(self, state_1, state_2):
#         x_1, y_1, alpha_1 = self.calc_landmark(state_1, self.Z1)
#         x_2, y_2, alpha_2 = self.calc_landmark(state_2, self.Z2)
#         self.e[0] = x_2 - x_1
#         self.e[1] = y_2 - y_1
#         self.e[2] = normalize(alpha_2 - alpha_1)
#
#         # r1 = self.calc_rotation_matrix(self.Z1[0, 1] + state_1[2, 0])
#         # r2 = self.calc_rotation_matrix(self.Z2[0, 1] + state_2[2, 0])
#         # self.omega = np.linalg.inv(r1 * self.P * r1.T + r2 * self.P * r2.T)
#         self.omega = np.eye(3)
#         self.state_1[:, :] = state_1[:, :]
#         self.state_2[:, :] = state_2[:, :]
#         return self.e.T * self.omega * self.e


def build_graph(zlist):
    print len(zlist)
    edges = []
    zids = list(itertools.combinations(range(len(zlist)), 2))
    for (t1, t2) in zids:
        if zlist[t1] is None or zlist[t2] is None:
            continue
        z_t1 = zlist[t1]
        z_t2 = zlist[t2]
        for iz1 in range(z_t1.shape[0]):
            for iz2 in range(z_t2.shape[0]):
                if int(z_t1[iz1, 3]) == int(z_t2[iz2, 3]):
                    # matched
                    edge = NoVEdge()
                    edge.setup(z_t1[iz1, 0], z_t1[iz1, 1], z_t1[iz1, 2], z_t2[iz2, 0], z_t2[iz2, 1], z_t2[iz2, 2], t1,
                               t2)
                    # edge = Edge(t1, z_t1[iz1, :], t2, z_t2[iz2, :], int(z_t1[iz1, 3]))
                    # edges[t1][t2].setdefault(z_t1[iz1, 2], edge)
                    edges.append(edge)
                    # ind = hash_index(t1, t2, len(zlist), z_t1[iz1, 3], 10)
                    # edges.setdefault(ind, edge)  # assume that there is no more than 10 landmarks.
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
    A, B = edge.jacobian()
    id1 = edge.t1 * 3
    id2 = edge.t2 * 3
    H[id1: id1 + 3, id1: id1 + 3] += A.T * edge.omega * A
    H[id1: id1 + 3, id2: id2 + 3] += A.T * edge.omega * B
    H[id2: id2 + 3, id1: id1 + 3] += B.T * edge.omega * A
    H[id2: id2 + 3, id2: id2 + 3] += B.T * edge.omega * B

    b[id1: id1 + 3, 0] += (A.T * edge.omega * edge.e)
    b[id2: id2 + 3, 0] += (B.T * edge.omega * edge.e)
    return H, b


def update_edges(edges, x_opt):
    cost = 0
    print x_opt.shape
    for edge in edges:
        # cost_i = edge.update(x_opt[:, edge.ida],x_opt[:, edge.idb])
        cost_i = edge.update(x_opt[0, edge.t1], x_opt[1, edge.t1], x_opt[2, edge.t1], x_opt[0, edge.t2],
                             x_opt[1, edge.t2], x_opt[2, edge.t2])
        cost += cost_i
    print "cost {}, edges {}".format(cost, len(edges))
    return edges


def calc_edges(xlist, zlist):
    edges = []
    cost = 0.0
    zids = list(itertools.combinations(range(len(zlist)), 2))
    for (t1, t2) in zids:
        x1, y1, yaw1 = xlist[0, t1], xlist[1, t1], xlist[2, t1]
        x2, y2, yaw2 = xlist[0, t2], xlist[1, t2], xlist[2, t2]
        if zlist[t1] is None or zlist[t2] is None:
            continue
        for iz1 in range(zlist[t1].shape[0]):
            for iz2 in range(zlist[t2].shape[0]):
                if int(zlist[t1][iz1, 3]) == int(zlist[t2][iz2, 3]):
                    d1 = zlist[t1][iz1, 0]
                    angle1, phi1 = zlist[t1][iz1, 1], zlist[t1][iz1, 2]
                    d2 = zlist[t2][iz2, 0]
                    angle2, phi2 = zlist[t2][iz2, 1], zlist[t2][iz2, 2]
                    edge = NoVEdge()
                    edge.setup(d1, angle1, phi1, d2, angle2, phi2, t1, t2)
                    c = edge.update(x1, y1, yaw1, x2, y2, yaw2)
                    edges.append(edge)
                    cost += c
    print ("cost: ", cost, "number of edges: ", len(edges))
    return edges


def graph_based_slam(XList, ZList, max_iter=20):
    x_opt = copy.deepcopy(XList)
    nx = x_opt.shape[1]
    n = nx * x_opt.shape[0]
    zlist = copy.deepcopy(ZList)
    zlist.insert(1, zlist[0])
    edges = build_graph(zlist)
    # edges = calc_edges(x_opt, zlist)

    for iter in range(max_iter):
        H = np.matrix(np.zeros((n, n)))
        b = np.matrix(np.zeros((n, 1)))
        cost = 0
        for edge in edges:
            x1, y1, yaw1 = x_opt[0, edge.t1], x_opt[1, edge.t1], x_opt[2, edge.t1]
            x2, y2, yaw2 = x_opt[0, edge.t2], x_opt[1, edge.t2], x_opt[2, edge.t2]
            c = edge.update(x1, y1, yaw1, x2, y2, yaw2)
            # cost_i = edge.update(x_opt[:, edge.t1], x_opt[:, edge.t2])
            cost += c
            H, b = fill_H_and_b(H, b, edge)
        print "cost = {}".format(cost)
        H[0:3, 0:3] += np.identity(3)
        # H[:, :] += np.identity(n)
        dx = -np.linalg.inv(H).dot(b)
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
    room.add_landmark(10.0, -2.0, np.radians(0))
    room.add_landmark(15.0, 10.0, np.radians(0))
    room.add_landmark(3.0, 15.0, np.radians(0))
    room.add_landmark(-5.0, 20.0, np.radians(0))
    room.add_landmark(-5.0, 5.0, np.radians(0))

    car = Car()

    show_graph_time = 20
    hxDR = np.matrix(np.zeros((3, 1)))
    hxTrue = np.matrix(np.zeros((3, 1)))
    hz = []
    dtime = 0
    step = 0
    xDR = np.matrix(np.zeros((3, 1)))

    while step < 100:
        step += 2.0
        dtime += 2.0

        car.move(dt=2.0)
        u = car.get_input()
        hxTrue = np.hstack((hxTrue, car.get_state()))
        xDR = estimate(xDR, u, 2.0)
        hxDR = np.hstack((hxDR, xDR))
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


def calc_input():
    v = 1.0
    yawrate = 0.1
    u = np.matrix([v, yawrate]).T
    return u


Qsim = np.diag([0.2, math.radians(1.0)]) ** 2
Rsim = np.diag([0.1, math.radians(10.0)]) ** 2
DT = 2.0

def motion_model(x, u):
    F = np.matrix([[1.0, 0, 0],
                   [0, 1.0, 0],
                   [0, 0, 1.0]])
    B = np.matrix([[DT * math.cos(x[2, 0]), 0],
                   [DT * math.sin(x[2, 0]), 0],
                   [0.0, DT]])
    return F * x + B * u


def observation(xTrue, xd, u, RFID):
    MAX_RANGE = 30.0
    xTrue = motion_model(xTrue, u)

    z = np.matrix(np.zeros((0, 4)))
    for i in range(RFID.shape[0]):
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx ** 2 + dy ** 2)
        angle = normalize(math.atan2(dy, dx)) - xTrue[2, 0]
        phi = RFID[i, 2] - (angle + xTrue[2, 0])
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Qsim[0, 0]
            anglen = angle + np.random.randn() * Qsim[1, 1]
            zi = np.matrix([dn, anglen, phi, i])
            z = np.vstack((z, zi))
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.matrix([ud1, ud2]).T
    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud


def main2():
    time = 0
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0],
                     [-5.0, 20.0, 0.0],
                     [-5.0, 5.0, 0.0]])
    STATE_SIZE = 3
    SIM_TIME = 200
    DT = 2
    xTrue = np.matrix(np.zeros((STATE_SIZE, 1)))
    xDR = np.matrix(np.zeros((STATE_SIZE, 1)))

    hxTrue = xTrue
    hxDR = xTrue
    hz = []
    dtime = 0.0
    show_graph_dtime = 20.0

    while SIM_TIME >= time:
        time += DT
        dtime += DT
        u = calc_input()
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz.append(z)
        if dtime >= show_graph_dtime:
            x_opt = graph_based_slam(hxDR, hz)
            dtime = 0.0

            plt.cla()
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxDR[0, :]).flatten(),
                     np.array(hxDR[1, :]).flatten(), "-k")
            plt.plot(np.array(x_opt[0, :]).flatten(),
                     np.array(x_opt[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time" + str(time)[0:5])
            plt.pause(1.0)


if __name__ == '__main__':
    main2()
