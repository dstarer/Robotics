#! env/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg

class Car:
    def __init__(self, x = 0.0, y = 0.0, yaw=0.0, v = 0.0, L=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L
        self.observe_variance = np.diag([0.4, 0.4]) ** 2
        self.process_variance = np.diag([1.0, math.radians(10)]) ** 2
        self.a = 1.0
        self.sigma = 0.1

    def _calc_input(self):
        self.a = 1.0
        self.sigma = 0.1
        return self.a, self.sigma

    def move(self, dt=0.01):
        a, sigma = self._calc_input()
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + sigma * dt
        self.v = a

    def observe(self):
        x = self.x + np.random.randn() * self.observe_variance[0, 0]
        y = self.y + np.random.randn() * self.observe_variance[1, 1]
        return x, y

    def get_input(self):
        a = self.a + np.random.randn() * self.process_variance[0, 0]
        sigma = self.sigma + np.random.randn() * self.process_variance[1, 1]
        return a, sigma

    def state(self):
        return self.x, self.y, self.yaw, self.v


class PositionEstimator:
    def __init__(self, initial_state, initial_P):
        """
        :param initial_state:
        """
        self.X = initial_state
        self.P = initial_P

    def jacob_F(self, x, u, DT=0.01):
        """

        :param x:
        :param u:
        :return:
        """
        yaw = x[2, 0]
        v = u[0, 0]
        j_f = np.matrix([
            [1.0, 0.0, - DT * math.sin(yaw), DT*math.cos(yaw)],
            [0.0, 1.0, DT * math.cos(yaw), DT*math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        return j_f

    def prediction(self, u, Q, DT=0.01):
        F = np.matrix('''
                1. 0. 0. 0.;
                0. 1. 0. 0.;
                0. 0. 1. 0.;
                0. 0. 0. 0.''')
        B = np.matrix([[DT * math.cos(self.X[2, 0]), 0], [DT * math.sin(self.X[2, 0]), 0], [0.0, DT], [1.0, 0]])
        X_prd = F * self.X + B * u
        j_f = self.jacob_F(self.X, u, DT=DT)
        self.P = j_f * self.P * j_f.T + Q
        self.X = X_prd
        return self.X, self.P

    def jacob_H(self, x):
        j_h = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0]])
        return j_h

    def estimation(self, z, R):
        j_h = self.jacob_H(self.X)
        y = z - j_h * self.X
        S = j_h * self.P * j_h.T + R
        K = self.P * j_h.T * np.linalg.inv(S)

        self.X = self.X + K * y
        self.P = (np.eye(self.X.shape[0]) - K * j_h) * self.P
        return self.X, self.P

    def ekf_estimation(self, u, z, Q, R, DT=0.01):
        self.prediction(u, Q, DT=DT)
        self.estimation(z, R)
        return self.X, self.P


class UKFPoseEstimator(object):
    def __init__(self, initial_state, initial_p, ALPHA, BETA, KAPPA):
        self.X = initial_state
        self.P = initial_p
        self.wm, self.wc, self.gamma = self._setup(self.X.shape[0], ALPHA, BETA, KAPPA)

    def _setup(self, n, ALPHA, BETA, KAPPA):
        lamda = ALPHA ** 2 * (n + KAPPA) - n
        wm = [lamda / (lamda + n)]
        wc = [lamda / (lamda + n) + (1 - ALPHA ** 2 + BETA)]

        for i in range(2 * n):
            wm.append(1.0 / (2 * (n + lamda)))
            wc.append(1.0 / (2 * (n + lamda)))

        gamma = math.sqrt(n + lamda)
        wm = np.matrix(wm)
        wc = np.matrix(wc)
        return wm, wc, gamma

    def _generate_sigma_points(self, gamma):
        sigma = self.X
        P_sqrt = np.matrix(scipy.linalg.sqrtm(self.P))
        n = self.X.shape[0]

        for i in range(n):
            sigma = np.hstack((sigma, self.X + gamma * P_sqrt[:, i]))

        for i in range(n):
            sigma = np.hstack((sigma, self.X - gamma * P_sqrt[:, i]))
        return sigma

    def _predict_motion(self, sigma, u):
        for i in range(sigma.shape[1]):
            sigma[:, i] = self._motion_model(sigma[:, i], u)
        return sigma

    def _motion_model(self, X, u, DT=0.1):
        F = np.matrix('''
            1. 0. 0. 0.;
            0. 1. 0. 0.;
            0. 0. 1. 0.;
            0. 0. 0. 0.
            ''')
        B = np.matrix([[DT * math.cos(X[2,0]), 0],
                       [DT * math.sin(X[2,0]), 0],
                       [0, DT],
                       [1, 0]])
        X_prd = F * X + B * u
        return X_prd

    def _observation_model(self, X):
        H = np.matrix('''
            1. 0. 0. 0.;
            0. 1. 0. 0.
        ''')
        return H * X

    def _calc_sigma_covariance(self, X, sigma, Q):
        nSigma = sigma.shape[1]
        d = sigma - X[0:sigma.shape[0], :]
        P = Q
        for i in range(nSigma):
            P = P + self.wc[0, i] * d[:, i] * d[:, i].T
        return P

    def _calc_p_xz(self, sigma, x, z_sigma, zb):
        nSigma = sigma.shape[1]
        dx = np.matrix(sigma - x)
        dz = np.matrix(z_sigma - zb[0:2, :])
        P = np.matrix(np.zeros((dx.shape[0], dz.shape[0])))
        for i in range(nSigma):
            P = P + self.wc[0, i] * dx[:, i] * dz[:, i].T
        return P

    def predict(self, u, Q, DT=0.1):
        sigma = self._generate_sigma_points(self.gamma)
        for i in range(sigma.shape[1]):
            sigma[:, i] = self._motion_model(sigma[:, i], u, DT=DT)
        self.X = (self.wm * sigma.T).T
        self.P = self._calc_sigma_covariance(self.X, sigma, Q)
        return self.X, self.P

    def estimation(self, z, R):
        z_pred = self._observation_model(self.X)
        y = z - z_pred
        # calculate K
        sigma = self._generate_sigma_points(self.gamma)
        # mean
        zb = (self.wm * sigma.T).T

        # to observation points
        for i in range(sigma.shape[1]):
            sigma[0:2, i] = self._observation_model(sigma[:, i])
        z_sigma = sigma[0:2, :]

        st = self._calc_sigma_covariance(zb, z_sigma, R)

        Pxz = self._calc_p_xz(sigma, self.X, z_sigma, zb)
        K = Pxz * np.linalg.inv(st)
        self.X = self.X + K * y
        self.P = self.P - K * st * K.T
        return self.X, self.P


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
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
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
    car = Car()
    initial_state = np.matrix(car.state()).T
    initial_P = np.diag([0.1, 0.1, 0.1, 0.1])

    Q = np.diag([0.01, 0.01, 0.01, 0.01])
    R = np.diag([1, 1])
    Q_ref = Q
    step = 500
    hxUkfEst = initial_state
    hxEkfEst = initial_state
    hxTrue = np.matrix(car.state()).T
    hz = np.matrix(car.observe()).T

    estimator = PositionEstimator(initial_state, initial_P)
    ukf_estimator = UKFPoseEstimator(initial_state, initial_P, 0.001, 2, 0)
    count = 0
    for i in range(step):
        car.move(dt=0.1)
        u = np.matrix(car.get_input()).T
        z = np.matrix(car.observe()).T
        x_true = np.matrix(car.state()).T
        x_ekf_est, p_ekf_est = estimator.prediction(u, Q_ref, DT=0.1)
        x_ukf_est, p_ukf_est = ukf_estimator.predict(u, Q_ref, DT=0.1)
        Q_ref = Q * 1.1
        count += 1
        if count == 10:
            x_ekf_est, p_ekf_est = estimator.estimation(z, R)
            x_ukf_est, p_ukf_est = ukf_estimator.estimation(z, R)
            Q_ref = Q
            # x_est, p_est = estimator.ekf_estimation(u, z, Q, R)
            count = 0

        hxEkfEst = np.hstack((hxEkfEst, x_ekf_est))
        hxUkfEst = np.hstack((hxUkfEst, x_ukf_est))
        hxTrue = np.hstack((hxTrue, x_true))
        hz = np.hstack((hz, z))

        plt.cla()
        plt.plot(hz[:, 0], hz[:, 1], "g+")
        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b", "real state")
        plt.plot(np.array(hxEkfEst[0, :]).flatten(),
                 np.array(hxEkfEst[1, :]).flatten(), "-r", "ekf state")
        plt.plot(np.array(hxUkfEst[0, :]).flatten(),
                 np.array(hxUkfEst[1, :]).flatten(), "-k", "ukf state")
        # plot_covariance_ellipse(x_est, p_est)
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)
    plt.show()


if __name__ == '__main__':
    main()