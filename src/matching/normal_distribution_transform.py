#!venv/bin/python
# -*- utf-8 -*-

import numpy as np
import math
import random
import matplotlib.pyplot as plt


class Voxel(object):
    def __init__(self, resX=0.5, resY=0.5):
        self.resX = resX
        self.resY = resY


class NDT(object):
    def __init__(self):
        self._is_convergence = False
        self.trans = np.eye(2)
        self.maxIter = 150
        self.target = None
        self.source = None

    def set_input_source(self, point_cloud):
        self.target = point_cloud

    def set_input_target(self, point_cloud):
        self.source = point_cloud

    def init_guess(self, theta, transformation):
        self.trans = transformation

    def align(self, maxIter=150):
        it = 0
        while it < maxIter:
            it += 1
            new_point_cloud = self.source * self.trans
            # todo calculate the delta_trans
            # try to calculate the score.
            # compute the jabocian H, and g

            # todo check whether is convergence.


    def get_final_transform(self):
        pass

    def is_convergence(self):
        pass
