#!venv/bin/python
# -*-encoding:utf-8-*-
"""
implement the Rapidly-Exploring Trees
using first order taylor expansion.
x' = f(x, u)
x_next = x + f(x, u) * dt

Generate_RRT(X_init, K, delta_t):
    initial Graph G
    G.add_vertex(X_init)

    for k = 1 to K do:
        X_rand = RANDOM_STATE()
        search nearest neighbor X_nearest of X_rand in G.
        calculate the differences u between X_rand and X_nearest # control the car move from X_nearest to X_rand
        calculate the new state X_new = f(X_near, u, delta_t)
        G.add_vertex(X_new)
        G.add_edge(X_near, X_new, u);
"""

import matplotlib.pyplot as plt
import random
import math
import copy


class Node(object):
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None

    @property
    def parent(self):
        return self.parent
    @parent.setter
    def parent(self, p):
        if isinstance(p, Node):
            self.parent = p
        raise ValueError("Non-compatible value type")


class RRT(object):
    def __init__(self, start, goal, obstacleList, randArea, dt=1.0, goalSampleRate=5, maxIter=5):
        """

        :param start: start postion (x, y)
        :param goal: goal position (x, y)
        :param obstacleList: obstacles [[x, y, size], ..., ]  treat each obstacle as one circle.
        :param randArea: Random Sampling Area [min, max]
        :param dt:
        :param goalSampleRate:
        :param maxIter:
        """
        self.start = start
        self.goal = goal
        self.minRand, self.maxRand = randArea
        self.expandDis = dt
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.G = []

    def planning(self):
        """
        :return:
        """
        self.G.append(Node(self.start[0], self.start[1]))
        while True:
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minRand, self.maxRand), random.uniform(self.minRand, self.maxRand)]
            else:
                rnd = [self.goal[0], self.goal[1]]
            ind = self.find_nearest_index(rnd)

    def find_nearest_index(self, pos):
        for node in self.G:
            pass
