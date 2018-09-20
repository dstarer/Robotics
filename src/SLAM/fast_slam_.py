#! venv/bin/python
# -*- encoding:utf-8 -*-

'''
The goal of slam is to get an accuracy state of robot.
And build the map according to the accuracy state.
For fast slam, replace the Robot state estimation method with particle filter
the mainly steps are as follows:
1. random particles around the initial state.
2. move-motion:
    for each particle, do state predict according to the move instruction and kinematic model.
    use the mean of particles as the robot's prediction state.

3. observation:
    for each particle, use extend-kalman filter calc the covariance of each landmark, and the estimates the state.
    Then, we can get mean of particles as the robot's estimation state.

4. resample
    according to the particles'weight, do resample.
    The resample process just replaces invalid particles.

Notice:
    1. weight computation:
        for new landmark, just add it to state and set the position and variance
        for landmark ever seen, just calculate the probabilty of observed in the pdf (X_lm, P_lm) where X_lm is the landmark's postion, and P_lm is the covariance.
    2. weight normalization process, the weight of one particle maybe very small, in this case, we should reset the weight to  1./p_n
'''

import numpy as np
import matplotlib.pyplot as plt
import math
import random


def normalize(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += math.pi * 2
    return angle


class Room(object):
    def __init__(self):
        self.landmarks = []

    def add_landmark(self, x, y):
        self.landmarks.append([x, y])

    def get_landmarks_as_matrix(self):
        return np.matrix(self.landmarks).T

    def get_landmarks(self):
        return self.landmarks

    def get_number_of_landmarks(self):
        return len(self.landmarks)


class Car(object):
    OBSERVE_DISTANCE = 20
    """
    The real car, moves in room
    suppose the car moves with fixed speed.
    model:
    x = x + v * cos(yaw) * dt
    y = y + v * sin(yaw) * dt
    yaw = yaw + delta * dt

    control vector:
    [v, delta]
    """

    def __init__(self, x=0, y=0, yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0
        self.delta = 0
        self.process_variance = np.matrix(np.diag([0.1, np.radians(1)])) ** 2
        self.observe_variance = np.matrix(np.diag([0.1, np.radians(1)])) ** 2

    def move(self, v=2, delta=np.radians(10), dt=0.1):
        self.x = self.x + v * dt * math.cos(self.yaw)
        self.y = self.y + v * dt * math.sin(self.yaw)
        self.yaw = self.yaw + delta * dt
        self.v = v + np.random.randn() * self.process_variance[0, 0]
        self.delta = delta + np.random.randn() * self.process_variance[1, 1]

    def get_input(self):
        """
        [a, delta].T
        :return:
        """
        return np.matrix([self.v, self.delta]).T

    def get_state(self):
        """
        [x, y, yaw].T
        :return:
        """
        return np.matrix([self.x, self.y, self.yaw]).T

    def observe(self, room):
        """
        suppose the car is equipped with radar.
        :param room:
        :type room: Room
        :return: [[d, theta, label], ...]
        """
        observed = []
        i = 0
        for lx, ly in room.landmarks:
            dx = lx - self.x
            dy = ly - self.y
            d = math.sqrt(dx * dx + dy * dy)
            theta = math.atan2(dy, dx)
            if d < self.OBSERVE_DISTANCE:
                d = d + np.random.randn() * self.observe_variance[0, 0]
                theta = theta + np.random.randn() * self.observe_variance[1, 1]
                observed.append([d, theta, i])
            i += 1

        return np.matrix(observed)


class Particle:

    def __init__(self, initial_X, landmark_size, landmark_number, copy=False):
        """
        :param initial_X:
        :type initial_X: [x, y, yaw, v]
        X = [x, y, theta, v, l1_x, l1_y, l2_x, l2_y, ...].T
        """
        if not copy:
            self.RS = len(initial_X)
            self.LS = landmark_size
            self.n_landmark = landmark_number
            self.X_r = np.matrix(initial_X).T
            self.X = np.matrix(np.zeros((landmark_number, landmark_size)))
            self.LP = np.matrix(np.zeros((landmark_number * self.LS, self.LS)))

    def state(self):
        return self.X_r

    def make_copy(self):
        o = Particle(self.X_r, self.LS, self.n_landmark, copy=True)
        o.RS = self.RS
        o.LS = self.LS
        o.X_r = self.X_r.copy()
        o.n_landmark = self.n_landmark
        o.X = self.X.copy()
        o.LP = self.LP.copy()
        return o

    def _get_nth_landmark_state(self, n):
        return self.X[n, :].T

    def _get_nth_landmark_covariance(self, n):
        start_pos = n * self.LS
        end_pos = start_pos + self.LS
        return self.LP[start_pos: end_pos, :]

    def move_motion(self, u, dt):
        """

        :param u: [a, delta].T
        :param Q: np.matrix(np.diag([std_a, std_delta]))
        :param dt:
        :return:
        """
        F = np.matrix([[1.0, 0.0, 0],
                       [0.0, 1.0, 0],
                       [0.0, 0.0, 1.0],
                       ])
        B = np.matrix([[dt * math.cos(self.X_r[2, 0]), 0.0],
                       [dt * math.sin(self.X_r[2, 0]), 0.0],
                       [0.0, dt]])
        self.X_r[2, 0] = normalize(self.X_r[2, 0])
        self.X_r = F * self.X_r + B * u

    def update(self, Z, R):
        """
        :param z: [[d_1, theta_1],
                    ...,
                    [d_i, theta_i],
                    ...,
                    [d_n, theta_n]]
        :type z: np.matrix
        :return:
        """
        w = 1
        observed_n = Z.shape[0]
        for i in range(observed_n):
            w *= self.update_one_landmark(Z[i, :].T, R)

        return w

    def update_one_landmark(self, z, R):
        """
        :param z: [d, theta].T
        :param R:
        :return:
        """
        w = 1.
        lm_id = int(z[2, 0])
        if abs(self.X[lm_id, 0]) < 0.01:
            self._add_landmark(z, R)
        else:
            w = self._compute_weight(z, R)

            residual, Hx, Hlm, Slm = self._calc_innovation(lm_id, z, R)
            K = self._get_nth_landmark_covariance(lm_id) * Hlm.T * np.linalg.inv(Slm)

            self.X[lm_id, :] = self.X[lm_id,:] + (K * residual).T

            start_pos = lm_id * self.LS
            end_pos = start_pos + self.LS
            P = self.LP[start_pos: end_pos, :]
            self.LP[start_pos: end_pos, :] = P - K * Hlm * P
        return w

    def _compute_weight(self, z, R):
        """
        measure the probability of residual between observed and calculated
        :param z:
        :param R:
        :return:
        """
        lm_id = int(z[2, 0])
        X_L = [self.X_r[0, 0] + z[0, 0] * math.cos(z[1, 0]), self.X_r[1, 0] + z[0, 0] * math.sin(z[1, 0])]
        X_L = np.matrix(X_L).T
        residual = X_L - self.X[lm_id, :].T

        # residual, Hx, Hlm, Slm = self._calc_innovation(lm_id, z, R)
        P = self._get_nth_landmark_covariance(lm_id)
        try :
            invP = np.linalg.inv(P)
        except np.linalg.linalg.LinAlgError:
            print ("singular")
            return 1.

        num = math.exp(-0.5 * residual.T * invP * residual)
        den = 2.0 * math.pi * math.sqrt(np.linalg.det(P))
        return num / den

    def _add_landmark(self, z, R):
        """
        :param z:
        :param R:
        :return:
        """
        X_L = [self.X_r[0, 0] + z[0, 0] * math.cos(z[1, 0]), self.X_r[1, 0] + z[0, 0] * math.sin(z[1, 0])]
        X_L = np.matrix(X_L)
        lm_id = int(z[2, 0])
        self.X[lm_id, :] = X_L
        # X_LM = H(R, z)
        # so, P = G_R * P_R_R * G_R.T + G_z * R * G_z.T
        G_r = np.matrix([[math.cos(z[1, 0]), -z[0, 0] * math.sin(z[1, 0])],
                         [math.sin(z[1, 0]), z[0, 0] * math.cos(z[1, 0])]])
        P = G_r * R * G_r.T
        start_pos = lm_id * self.LS
        end_pos = start_pos + self.LS
        self.LP[start_pos: end_pos, :] = P

    def _calc_innovation(self, n, z_observed, R):
        dx = self.X[n, 0] - self.X_r[0, 0]
        dy = self.X[n, 1] - self.X_r[1, 0]

        square_distance = dx * dx + dy * dy
        z = np.matrix([math.sqrt(square_distance), math.atan2(dy, dx)]).T

        residual = z_observed[:2, :] - z
        residual[1, 0] = normalize(residual[1, 0])

        Hx, Hlm = self._observation_jacob(square_distance, z[0, 0], dx, dy)
        Slm = Hlm * self._get_nth_landmark_covariance(n) * Hlm.T + R
        return residual, Hx, Hlm, Slm

    def _observation_jacob(self, square_distance, distance, dx, dy):
        Hx = np.matrix([[-dx / distance, -dy / distance, 0],
                        [dy / square_distance, -dx / square_distance, 0]])
        Hlm = np.matrix([[dx / distance, dy / distance],
                         [-dy / square_distance, dx / square_distance]])
        return Hx, Hlm


class ParticleSlam(object):
    """
    do pose estimation and mapping
    Todo: how to get the landmark's position.
    """
    def __init__(self, p_n, l_n):
        self.p_n = p_n
        self.l_n = l_n
        self.RS = 3
        self.LS = 2
        self.pw = np.zeros((p_n, 1)) + 1. / p_n
        self.particles = [Particle([0.] * self.RS, self.LS, l_n) for i in range(p_n)]
        self.NTH = p_n / 2

    def predict(self, u, Q, dt):
        """
        :param u:
        :param Q:
        :param dt:
        :return:
        """
        for p in self.particles:
            u_p = np.zeros((2, 1))
            u_p[0, 0] = u[0, 0] + Q[0, 0] * np.random.randn()
            u_p[1, 0] = u[1, 0] + Q[1, 1] * np.random.randn()
            p.move_motion(u_p, dt)
        return self._estimate_state()

    def update(self, Z, R):
        """
        :param Z:
        :param R:
        :return:
        """
        for i, p in enumerate(self.particles):
            w = p.update(Z, R)
            self.pw[i, 0] = w
        self._resample()
        return self._estimate_state()

    def _generate_random(self):
        rp = []
        for i in range(self.p_n):
            rp.append(random.uniform(0, 1))
        return rp

    def _resample(self):
        """
        :return:
        """
        self._normalize()
        need_resample = False
        try:
            t = (self.pw.T * self.pw)[0, 0]
            if t < 1e-6:
                need_resample=True
            else:
                Neff = 1./ t
                if Neff < self.NTH:
                    need_resample = True
        except ZeroDivisionError:
            need_resample = True

        if need_resample:
            print("do resample")
            wcum = np.cumsum(self.pw)
            wcum[-1] = 1.

            rp = self._generate_random()
            # try to find the more important particles.
            indexes = []
            for p in rp:
                ind = 0
                while wcum[ind] < p:
                    ind += 1
                indexes.append(ind)

            new_particles = []
            for ind in indexes:
                new_particles.append(self.particles[ind].make_copy())

            self.particles = new_particles
            self._normalize()

    def _normalize(self):
        ws = self.pw.sum()
        try:
            self.pw = self.pw / ws
        except ZeroDivisionError:
            self.pw = np.zeros((self.p_n, 1)) + 1. / self.p_n

    def _estimate_state(self):
        X = np.zeros((self.RS, 1))
        for i, p in enumerate(self.particles):
            X = X + self.pw[i, 0] * p.X_r
        return X

    def get_particles_pose(self):
        X = np.zeros((3, 0))
        for p in self.particles:
            X = np.hstack((X, p.X_r))
        return X


def main():
    room = Room()
    room.add_landmark(10.0, -2.0)
    room.add_landmark(15.0, 10.0)
    room.add_landmark(15.0, 15.0)
    room.add_landmark(10.0, 20.0)
    room.add_landmark(3.0, 15.0)
    room.add_landmark(-5.0, 20.0)
    room.add_landmark(-5.0, 5.0)
    room.add_landmark(-10.0, 15.0)

    car = Car()
    p_n = 50

    slam = ParticleSlam(p_n, room.get_number_of_landmarks())

    Q = np.matrix(np.diag([0.1, np.radians(5)]))
    R = np.matrix(np.diag([0.1, np.radians(5)]))

    hxEst = np.matrix(np.zeros((3, 1)))
    hxTrue = np.matrix(np.zeros((3, 1)))

    dt = 0.1
    t = 0
    error = []
    while t < 30:
        t += dt
        car.move(dt=dt)
        u = car.get_input()
        Z = car.observe(room)

        slam.predict(u, Q, dt)
        X_est = slam.update(Z, R)
        X_true = car.get_state()
        hxEst = np.hstack((hxEst, X_est))
        hxTrue = np.hstack((hxTrue, X_true))

        plt.cla()

        # for i in range(len(z_observed)):
        #     plt.plot([X_true[0, 0], z_observed[i][1]], [X_true[1, 0], z_observed[i][2]], "-k")
        error.append((X_est[0, 0] - X_true[0, 0], X_est[1, 0] - X_true[1, 0]))

        for landmark in room.get_landmarks():
            plt.plot(landmark[0], landmark[1], "*k")
        P_X = slam.get_particles_pose()
        plt.plot(P_X[0, :], P_X[1, :], ".r")

        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b")

        plt.plot(np.array(hxEst[0, :]).flatten(),
                 np.array(hxEst[1, :]).flatten(), "-r")
        plt.pause(0.001)

    n = len(error)
    X_error = [e[0] for e in error]
    Y_error = [e[1] for e in error]
    d_error = [math.sqrt(e[0] * e[0] + e[1] * e[1]) for e in error]
    ind = range(n)

    plt.figure()
    plt.plot(ind, X_error, "b-", label="error_x")
    plt.plot(ind, Y_error, "g-", label="error_y")
    plt.plot(ind, d_error, "y-", label="error_dist")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()