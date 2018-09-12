#! env/bin/pyhton
# -*-encoding:utf-8-*-

import math
import sys
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("../PathPlanning/")
import cubic_spline_planner


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, L=1.9):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L

    def update(self, a, sigma, dt=0.1):
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.L * math.tan(sigma) * dt
        self.v = self.v + a * dt
        return self.x, self.y, self.yaw, self.v

    def get_front(self):
        fx = self.x + self.L * np.cos(self.yaw)
        fy = self.y + self.L * np.sin(self.yaw)
        return fx, fy


def normalize_angle(angle):
    while angle > math.pi:
        angle -= math.pi * 2

    while angle < -math.pi:
        angle += math.pi

    return angle


def longitude_control(v_ref, v, Kp=1.0):
    return Kp * (v_ref - v)


def latitude_control(state, cx, cy, cyaw, last_target_idx, k=1.0):
    """

    :param state:
    :param cx:
    :param cy:
    :param cyaw:
    :param last_target_idx:
    :return:
    """
    current_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx > current_idx:
        current_idx = last_target_idx

    theta_e = normalize_angle(cyaw[current_idx] - state.yaw)

    theta_d = np.arctan2(k * error_front_axle, state.v)

    delta = theta_e + theta_d
    return delta, current_idx


def distance(fx, fy, x, y):
    return (fx - x) * (fx - x) + (fy - y) * (fy - y)


def calc_target_index(state, cx, cy):
    fx, fy = state.get_front()
    fd = [distance(fx, fy, x, y) for x, y in zip(cx, cy)]
    d_ref = min(fd)
    target_idx = fd.index(d_ref)

    target_yaw = normalize_angle(np.arctan2(fy - cy[target_idx], fx - cx[target_idx]) - state.yaw)
    if target_yaw > 0.0:
        d_ref = -d_ref
    return target_idx, d_ref


def main():
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = State(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)
    show_animation = True
    dt = 0.1
    while max_simulation_time >= time and last_idx > target_idx:
        ai = longitude_control(target_speed, state.v)
        di, target_idx = latitude_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()