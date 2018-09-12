#! venv/bin/python
# -*- encoding:utf-8 -*-

from car import Car
import matplotlib.pyplot as plt
import numpy as np


def p_control(car, tau_p, n=100, speed=1.0):
    """
    :param car:
    :param tau_p:
    :param n:
    :param speed:
    :return:
    """
    x_trj = []
    y_trj = []
    yaw_trj = []
    steering = 0.0
    for i in range(n):
        cte = car.y
        steering -= tau_p * cte
        car.move(steering, speed)
        x_trj.append(car.x)
        y_trj.append(car.y)
        yaw_trj.append(car.yaw)

    return x_trj, y_trj, yaw_trj


def p_i_control(car, tau_p, tau_i, n = 100, speed=1.0):
    x_trj, y_trj, yaw = [], [], []
    steering = 0.0
    icte = 0
    for i in range(n):
        cte = car.y
        icte += cte
        steering = - tau_p * cte - tau_i * icte
        car.move(steering, speed)
        x_trj.append(car.x)
        y_trj.append(car.y)
        yaw.append(car.yaw)
    return x_trj, y_trj, yaw


def p_d_control(car, tau_p, tau_d, n = 100, speed = 1.0):
    x_trj, y_trj, yaw = [], [], []
    steering = 0.0
    prev_cte = 0
    for i in  range(n):
        cte = car.y
        d_cte = cte - prev_cte
        prev_cte = cte
        steering = - tau_p * cte - tau_d * d_cte
        car.move(steering, speed)
        x_trj.append(car.x)
        y_trj.append(car.y)
        yaw.append(car.yaw)
    return x_trj, y_trj, yaw


def pid_control(car, tau_p, tau_d, tau_i, n= 100, speed=1.0):
    x_trj, y_trj, yaw = [], [], []
    steering = 0.0
    prev_cte = 0
    i_cte = 0
    for i in range(n):
        cte = car.y
        d_cte = cte - prev_cte
        i_cte = i_cte + cte
        prev_cte = cte
        steering = - tau_p * cte - tau_d * d_cte - i_cte * tau_i
        car.move(steering, speed)
        x_trj.append(car.x)
        y_trj.append(car.y)
        yaw.append(car.yaw)
    return x_trj, y_trj, yaw


car_1 = Car()
car_1.set(0, 1, 0)
car_2 = Car()
car_2.set(0, 1, 0)
car_3 = Car()
car_3.set(0, 1, 0)
car_4 = Car()
car_4.set(0, 1, 0)

p_x, p_y, p_yaw = p_control(car_1, 1)
p_i_x, p_i_y, p_i_yaw = p_i_control(car_2, 0.1, 0.0001)
p_d_x, p_d_y, p_d_yaw = p_d_control(car_3, 0.1, 1)
pid_x, pid_y, pid_yaw = pid_control(car_4, 0.1, 3.0, 0.0001)

n = len(p_x)
print n
plt.figure()
plt.plot(p_x, p_y, 'g', label='P Controller')
plt.plot(p_i_x, p_i_y, 'b', label='PI Controller')
# plt.plot(p_d_x, p_d_y, 'b', label='PD controller')
# plt.plot(pid_x, pid_y, 'p', label='PID controller')
plt.plot(np.array(range(n)), np.zeros(n), 'r', label='reference')
plt.legend()
plt.show()