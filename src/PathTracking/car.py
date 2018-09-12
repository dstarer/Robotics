#!/usr/bin/python env
# -*- encoding:utf-8 -*-

import random
import numpy as np


class Car(object):
    def __init__(self, x=0, y=0, yaw=0, length=20.0, steering_noise=0.0, distance_nise=0.0, steering_drift=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.length = length
        self.steering_noise = steering_noise
        self.distance_noise = distance_nise
        self.steering_drift = steering_drift

    def set(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw % (2.0 * np.pi)

    def set_noise(self, steering_noise, distance_noise):
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, steering_drift):
        self.steering_drift = steering_drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # add noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        steering2 += self.steering_drift # add sliding slope
        turn = np.tan(steering2) * distance2 / self.length
        # apply straight line motion
        if abs(turn) < tolerance:
            self.x += distance2 * np.cos(self.yaw)
            self.y += distance2 * np.sin(self.yaw)
            self.yaw = (self.yaw + turn) % (2.0 * np.pi)
        else:
            # bicycle model
            radius = distance2 / turn
            cx = self.x - (np.sin(self.yaw) * radius)
            cy = self.y + (np.cos(self.yaw) * radius)
            self.yaw = (self.yaw + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.yaw) * radius)
            self.y = cy - (np.cos(self.yaw) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f yaw=%.5f]' % (self.x, self.y, self.yaw)
