#!venv/bin/python
# -*- encoding:utf-8 -*-

import math
import numpy as np
import sys
import matplotlib.pyplot as plt
sys.path.append("../PathPlanning/")
import cubic_spline_planner


class Robot(object):
    def __init__(self, x=0, y=0, yaw=0, v=0, L=1.9):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.L = L

    def move(self, a, sigma, dt=0.1):
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.v / self.L * math.tan(sigma) * dt
        self.v = self.v + a * dt
        return self.x, self.y, self.yaw, self.v

    def get_state(self):
        return self.x, self.y, self.yaw, self.v


def normalize_angle(angle):
    while angle >= math.pi:
        angle -= math.pi * 2
    while angle < -math.pi:
        angle += math.pi * 2
    return angle


def longitude_control(v_target, v, KP=1.0):
    return (v_target - v) * KP


def latitude_control(state, target, L):
    """
    :param state: [x, y, yaw, v]
    :param target: [x, y, yaw, v]
    :return:
    """
    dx = target[0] - state[0]
    dy = target[1] - state[1]
    alpha = normalize_angle(math.atan2(dy, dx) - state[2])
    dist = math.sqrt(dx ** 2 + dy ** 2)
    return math.atan2(2 * L * math.sin(alpha) / dist, 1.0)


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def find_closest(state, lastIdx, waypoints):
    """
    :param state: [x, y, yaw, v]
    :param lastIdx:
    :param waypoints:
    :return:
    """
    dist = [distance(state, waypoints[i]) for i in range(lastIdx, len(waypoints))]
    closest = dist.index(min(dist)) + lastIdx
    return closest


def find_target(closest, waypoints, look_forward):
    """
    :param closest: int
    :param waypoints: [[x, y, yaw, v], [x, y, yaw, v], ...]
    :param look_forward: \ell_d
    :return:
    """
    look_ahead_distance = 0
    target = closest
    while look_ahead_distance < look_forward and target + 1 < len(waypoints):
        look_ahead_distance += distance(waypoints[target], waypoints[target + 1])
        target += 1
    if target >= len(waypoints):
        target = len(waypoints) - 1
    return target, look_ahead_distance


def main():
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    target_speed = 15.0 / 3.6

    max_simulation_time = 100.0

    waypoints = [[x, y, yaw, target_speed] for (x, y, yaw) in zip(cx, cy, cyaw)]
    car = Robot(x=0, y=0.5, yaw=np.radians(10.0), v=0.0)

    goal_idx = len(cx) - 1
    dtime = 0.0
    x = [car.x]
    y = [car.y]
    yaw = [car.yaw]
    v = [car.v]
    t = [0.0]
    steer = [0]
    dt = 0.1
    closest_idx = find_closest(car.get_state(), 0, waypoints)
    look_forward = 6.0
    while max_simulation_time >= dtime and goal_idx > closest_idx:
        closest_idx = find_closest(car.get_state(), closest_idx, waypoints)
        target_idx, look_ahead_distance = find_target(closest_idx, waypoints, look_forward)
        if closest_idx == target_idx or look_ahead_distance < look_forward / 2:
            break
        a = longitude_control(waypoints[target_idx][3], car.v)
        delta = latitude_control(car.get_state(), waypoints[target_idx], car.L)
        car.move(a, delta, dt=dt)
        steer.append(delta)
        dtime += 0.1
        x.append(car.x)
        y.append(car.y)
        yaw.append(car.yaw)
        v.append(car.v)
        t.append(dtime)

        plt.cla()
        plt.title("pure pursuit")
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        # plt.show()
        plt.pause(0.0001)


    plt.subplots(1)
    plt.plot(t, [iv * 3.6 for iv in v], "-r")
    plt.xlabel("Time[s]")
    plt.ylabel("Speed[km/h]")
    plt.grid(True)

    plt.subplots(1)
    plt.plot(t, [iy * 180 / math.pi for iy in steer], "-b")
    plt.xlabel("Time[s]")
    plt.ylabel("Steer[degree]")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()

