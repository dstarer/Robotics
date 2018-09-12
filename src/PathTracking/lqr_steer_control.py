#!env/bin/python
# -*- encoding:utf-8 -*-

import sys

sys.path.append("../PathPlanning/")

import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.linalg as la

import cubic_spline_planner

Q = np.eye(4)
R = np.eye(1)


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, L=0.5):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L

    def update(self, a, delta, max_steer=math.radians(45.0), dt=0.1):
        if delta > max_steer:
            delta = max_steer
        elif delta < -max_steer:
            delta = -max_steer

        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.L * math.tan(delta) * dt
        self.v = self.v + a * dt
        return self.x, self.y, self.v, self.yaw


def speed_control(target, current, KP = 1.0):
    a = KP * (target - current)
    return a


def pi_2_pi(angle):
    return (angle + math.pi) % ( 2 * math.pi) - math.pi


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0
    for i in range(len(cx) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed

        if switch:
            speed_profile[i] = 0.0

    speed_profile[-1] = 0.0
    return speed_profile


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx * idx + idy * idy for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)
    dx1 = cx[ind] - state.x
    dy1 = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dy1, dx1))
    if angle < 0:
        mind *= -1
    return ind, mind


def solve_DARE(A, B, Q, R, max_iter=150, eps=0.01):
    X = Q
    for i in range(max_iter):
        X_n = A.T * X * A - A.T * X * B * la.inv(R + B.T * X * B) * B.T * X * A + Q
        # X_n = A.T * X + Q - X * B * la.inv(R + B.T * X * B) * B.T * X
        if (abs(X_n - X)).max() < eps:
            X = X_n
            break
        X = X_n
    return X


def dlqr(A, B, Q, R):
    X = solve_DARE(A, B, Q, R)
    K = np.matrix(la.inv(B.T * X * B + R) * (B.T * X * A))
    # K = np.matrix(la.inv(B.T * X * B + R)) * B.T * X
    eigVals, eigVecs = la.eig(A - B * K)
    return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, dt=0.1):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)
    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    A = np.matrix(np.zeros((4, 4)))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.matrix(np.zeros((4, 1)))
    B[3, 0] = v / state.L

    K, _, _ = dlqr(A, B, Q, R)

    x = np.matrix(np.zeros((4, 1)))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(state.L * k, 1)
    fb = pi_2_pi((-K * x)[0, 0])

    delta = ff + fb

    return delta, ind, e, th_e


def control(cx, cy, cyaw, ck, speed_profile, goal, show_animation=True):
    T = 500.0
    goal_dis = 0.3
    stop_speed = 0.05
    state = State()

    time = 0.0
    x = [state.x]
    y = [state.y]
    v = [state.v]
    yaw = [state.yaw]
    t = [0.0]
    dt = 0.1

    target_ind = calc_nearest_index(state, cx, cy, cyaw)
    e, e_th = 0, 0.0

    while T >= time:
        dl, target_ind, e, e_th = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th)
        a_i = speed_control(speed_profile[target_ind], state.v)

        state.update(a_i, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt(dx * dx + dy * dy) <= goal_dis:
            print("Goal")
            break

        x.append(state.x)
        y.append(state.y)
        v.append(state.v)
        yaw.append(state.yaw)
        t.append(T)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]: " + str(round(state.v * 3.6, 2)) + ", target index:" + str(target_ind))
            plt.pause(0.0001)
    return t, x, y, v, yaw


def main():
    show_animation = True
    print("LQR steering control tracking start!!")
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=1.0)
    target_speed = 15.0 / 3.6  # simulation parameter km/h -> m/s

    sp = calc_speed_profile(cx, cy, cyaw, target_speed)

    t, x, y, v, yaw = control(cx, cy, cyaw, ck, sp, goal)

    if show_animation:
        plt.close()
        flg, _ = plt.subplots(1)
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        flg, ax = plt.subplots(1)
        plt.plot(s, [math.degrees(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        flg, ax = plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

        plt.show()


if __name__ == '__main__':
    main()