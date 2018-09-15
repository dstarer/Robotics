#! env/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy


def normalization(angle):
    while angle > math.pi:
        angle -= math.pi * 2
    while angle < -math.pi:
        angle += math.pi * 2
    return angle


class Room(object):
    def __init__(self):
        self.landmarks = []

    def add_landmark(self, x, y):
        self.landmarks.append([x, y])

    def get_landmarks(self):
        return self.landmarks


class Car:
    MAX_RANGE = 20.0
    def __init__(self, x = 0.0, y = 0.0, yaw=0.0, v = 1.0, p = np.diag([0.1, math.radians(5)]) ** 2, r = np.diag([0.4, 0.4]) ** 2):
        """
        x, y, yaw, v represents the initial car's state.
        P, R represents the initial process variance and observe variance.
        :param x:
        :param y:
        :param yaw:
        :param P:
        :param R:
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.observe_variance = r
        self.process_variance = p
        self.sigma = 0.1

    def _calc_input(self):
        self.sigma = 0.1
        self.v = 1.0
        return self.v, self.sigma

    def move(self, dt=0.01):
        a, sigma = self._calc_input()
        self.x = self.x + a * math.cos(self.yaw) * dt
        self.y = self.y + a * math.sin(self.yaw) * dt
        self.yaw = self.yaw + sigma * dt

    def observe(self, room):
        z = np.matrix(np.zeros((0, 3)))
        landmarks = room.get_landmarks()
        for i, landmark in enumerate(landmarks):
            dx = landmark[0] - self.x
            dy = landmark[1] - self.y
            d = math.sqrt(dx * dx + dy * dy)
            angle = normalization(math.atan2(dy, dx))
            if d <= self.MAX_RANGE:
                observe_d = d + np.random.randn() * self.observe_variance[0, 0]
                observe_angle = angle + np.random.randn() * self.observe_variance[1,1]
                zi = np.matrix([observe_d, observe_angle, i])
                z = np.vstack((z, zi))
        return z

    def get_input(self):
        sigma = self.sigma + np.random.randn() * self.process_variance[1,1]
        v = self.v + np.random.randn() * self.process_variance[0, 0]
        return v, sigma

    def state(self):
        return self.x, self.y, self.yaw, self.v


class Map(object):
    def __init__(self, robot_state_size, landmarks_state_size):
        self.X = np.zeros((robot_state_size, 1))
        self.P = np.ones((robot_state_size, robot_state_size))
        self.RS = robot_state_size
        self.LS = landmarks_state_size
        self.n_landmark = 0
        self.s = robot_state_size
        self.M_DIST_H = 2.0

    def get_landmarks(self):
        pos = []
        for i in range(self.n_landmark):
            pos.append(self.get_nth_landmark_state(i))
        return pos

    def _motion_model(self, u, dt=0.1):
        F = np.matrix([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])
        B = np.matrix([[dt * math.cos(self.X[2, 0]), 0], [dt * math.sin(self.X[2, 0]), 0], [0.0, dt]])
        self.X[0: self.RS] = F * self.X[0:self.RS] + B * u

    def _motion_jacob(self, u, dt=0.1):
        Fx = np.hstack((np.eye(self.RS), np.zeros((self.RS, self.n_landmark * self.LS))))
        jF = np.matrix([[0, 0, -dt * u[0] * math.sin(self.X[2, 0])],
                        [0, 0, dt * u[0] * math.cos(self.X[2, 0])],
                        [0, 0, 0]])
        G = Fx.T * jF * Fx
        return G, Fx

    def predict(self, u, Q, dt=0.1):
        """
        robot move u, and do predict according to the u.
        :param u:
        :return:
        """
        self._motion_model(u, dt)
        G, Fx = self._motion_jacob(u, dt)
        self.P = G * self.P * G.T + Fx.T * Q * Fx

    def ekf_slam(self, u, z, Q, R, dt=0.1):
        self.predict(u, Q, dt)

        for iz in z:
            self.update_one_landmark(iz, R)

        self.X[2] = normalization(self.X[2])

        return self.X, self.P

    def update_one_landmark(self, z, R):
        """
        for one give landmark, execute the ekf-update process.
        before update state, one should search the matching id for z
        :param z:
        :return:
        """
        matched_id = self._search_matched_id(z, R)
        if matched_id >= self.n_landmark:
            print("New Lm")
            X_new = np.vstack((self.X, self._calc_lm_pos(z)))
            P_new = np.vstack((np.hstack((self.P, np.zeros((self.s, self.LS)))),
                               np.hstack((np.zeros((self.LS, self.s)), np.eye(2)))))
            self.X = X_new
            self.P = P_new

            self.s += 2
            self.n_landmark += 1
        lm_state = self.get_nth_landmark_state(matched_id)
        y, S, H = self._calc_innovation(matched_id, lm_state, z, R)

        K = self.P * H.T * S
        self.X = self.X + K * y
        self.P = self.P - K * H * self.P

    def get_nth_landmark_state(self, n):
        return self.X[self.RS + n * self.LS: self.RS + (n + 1) * self.LS]

    def _search_matched_id(self, z, R):
        """
        Mahalanobis_distance is used to match two landmarks
        :param z:
        :return:
        """
        m_dist = []
        for i in range(self.n_landmark):
            landmark = self.get_nth_landmark_state(i)
            y, S, H = self._calc_innovation(i, landmark, z, R)
            m_dist.append(y.T  * np.linalg.inv(S) * y)
        m_dist.append(self.M_DIST_H)
        min_dist = min(m_dist)
        return m_dist.index(min_dist)

    def _calc_innovation(self, n, landmark, z, R):
        delta = landmark - self.X[:self.RS - 1]
        square_dist = (delta.T * delta)[0, 0]

        z_angle = math.atan2(delta[1], delta[0]) - self.X[2]
        z_observed = np.matrix([math.sqrt(square_dist), normalization(z_angle)])
        residual = (z[0, :2] - z_observed).T
        residual[1] = normalization(residual[1])

        H = self._observed_jacob(square_dist, delta, n + 1)
        S = H * self.P * H.T + R

        return residual, S, H

    def _calc_lm_pos(self, z):
        """
        calc landmark's position according to observation z
        x = R_X + dist * cos(theta)
        y = R_Y + dist * sin(theta)
        :param z:
        :return:
        """
        zp = np.zeros((2, 1))
        zp[0, 0] = self.X[0, 0] + z[0, 0] * math.cos(self.X[2, 0] + z[0, 1])
        zp[1, 0] = self.X[1, 0] + z[0, 0] * math.sin(self.X[2, 0] + z[0, 1])
        return zp

    def _observed_jacob(self, square_dist, delta_X, n):
        sq = math.sqrt(square_dist)
        # [d(h(r, s, lmi)) / d(r) , d(h(r, s, lmi)) / d (lmi)]
        G = np.matrix([[- sq * delta_X[0, 0], -sq * delta_X[1, 0], 0, sq * delta_X[0, 0], sq * delta_X[1, 0]],
                        [delta_X[1, 0], - delta_X[0, 0], -1.0, -delta_X[1, 0], delta_X[0, 0]]
                      ])
        G = G/square_dist
        F1 = np.hstack((np.eye(self.RS), np.zeros((self.RS, self.LS * self.n_landmark))))
        F2 = np.hstack((np.zeros((self.LS, self.RS)), np.zeros((self.LS, self.LS * (n - 1))),
                        np.eye(2), np.zeros((self.LS, self.LS * (self.n_landmark - n)))))
        F = np.vstack((F1, F2))
        H = G * F
        return H


def main():
    t = 0.0
    # construct room
    room = Room()
    room.add_landmark(-10.0, -2.0)
    room.add_landmark(15.0, 10.0)
    room.add_landmark(3.0, 15.0)
    room.add_landmark(-5.0, 20.0)
    # initial car's state
    car = Car()
    # initial slam map
    slam = Map(3, 2)

    Q = np.matrix(np.diag([0.01, 0.01, 0.01]))
    R = np.matrix(np.diag([1, 1]))

    hxTrue = np.matrix(np.zeros((3, 1)))
    hxEst = hxTrue
    dt = 0.1
    show_animation = True
    while t <= 100:
        t += dt
        # move the car and observe the room
        car.move(dt)
        z_observed = car.observe(room)
        v, sigma = car.get_input()
        u = np.matrix(np.array([v, sigma])).T
        x_est, p_est = slam.ekf_slam(u, z_observed, Q, R)

        _x, _y, _yaw, _v = car.state()
        hxTrue = np.hstack((hxTrue, np.matrix(np.array([_x, _y, _yaw])).T ))
        hxEst = np.hstack((hxEst, x_est[0:3]))

        if show_animation:
            plt.cla()
            landmarks = room.get_landmarks()
            landmarks_x = [p[0] for p in landmarks]
            landmarks_y = [p[1] for p in landmarks]

            plt.plot(landmarks_x, landmarks_y, "*k")
            plt.plot(x_est[0], x_est[1], ".r")

            landmarks_observed = slam.get_landmarks()
            # plot landmark
            for lm in landmarks_observed:
                plt.plot(lm[0], lm[1], "xg")

            plt.plot(np.array(hxTrue[0, :]).flatten(),
                     np.array(hxTrue[1, :]).flatten(), "-b")
            plt.plot(np.array(hxEst[0, :]).flatten(),
                     np.array(hxEst[1, :]).flatten(), "-r")
            plt.axis("equal")
            plt.grid(True)
        plt.pause(0.001)
    plt.pause(100000)

if __name__ == '__main__':
    main()
