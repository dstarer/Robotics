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
        self._parent = None

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, p):
        if isinstance(p, int):
            self._parent = p
            return
        raise ValueError("Non-compatible value type")


def is_collision(first, second, obstacleList):
    x1, y1 = first[0], first[1]
    x2, y2 = second[0], second[1]
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1
    mod_l = math.sqrt(a ** 2 + b ** 2)
    for (ox, oy, o_l) in obstacleList:
        d = abs(a * ox + b * oy + c) / mod_l
        if d <= o_l:
            return True
    return False


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

            new_node = self._move_towards(ind, rnd)
            # if is_collision([self.G[ind].x, self.G[ind].y], [new_node.x, new_node.y], self.obstacleList):
            #     continue
            if self._is_collision(new_node):
                continue
            self.G.append(new_node)

            # check goal
            d = math.sqrt((self.goal[0] - new_node.x) ** 2 + (self.goal[1] - new_node.y) ** 2)
            if d <= self.expandDis:
                print "Goal"
                break
            self.drawn_graph(rnd)

        path = [self.goal]
        ind = len(self.G) - 1
        while self.G[ind].parent is not None:
            cur = self.G[ind]
            path.append([cur.x, cur.y])
            ind = cur.parent
        path.append([self.start[0], self.start[1]])
        return path

    def _is_collision(self, node):
        for obstacle in self.obstacleList:
            d = math.sqrt((obstacle[0] - node.x) ** 2 + (obstacle[1] - node.y) ** 2)
            if d < obstacle[2]:
                return True
        return False

    def find_nearest_index(self, pos):
        dist = [(pos[0] - node.x) ** 2 + (pos[1] - node.y) ** 2 for node in self.G]
        return dist.index(min(dist))

    def _move_towards(self, ind, target):
        outset = self.G[ind]
        theta = math.atan2(target[1] - outset.y, target[0] - outset.x)
        n_x = outset.x + self.expandDis * math.cos(theta)
        n_y = outset.y + self.expandDis * math.sin(theta)
        new_node = Node(n_x, n_y)
        new_node.parent = ind
        return new_node

    def drawn_graph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.G:
            if node.parent is not None:
                plt.plot([node.x, self.G[node.parent].x], [node.y, self.G[node.parent].y], "-g")

        for (ox, oy, od) in self.obstacleList:
            plot_circle(ox, oy, od)

        plt.plot(self.start[0], self.start[1], "xr")
        plt.plot(self.goal[0], self.goal[1], "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.02)


def plot_circle(x, y, size):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    plt.plot(xl, yl, "-k")


def drawn_graph(path, obstacleList):
    plt.clf()

    for (ox, oy, od) in obstacleList:
        plot_circle(ox, oy, od)

    plt.plot([x for (x, y) in path], [y for (x, y) in path], "-b")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.02)


class PathPostProcessor(object):
    @staticmethod
    def calc_length(path):
        length = 0
        for i in range(len(path) - 1):
            d = math.sqrt((path[i + 1][0] - path[i][0]) ** 2 + (path[i + 1][1] - path[i][1]) ** 2)
            length += d
        return length

    @staticmethod
    def search_target_point(path, l):
        accum_l = 0
        ind = 0
        seg_l = 0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            d = math.sqrt(dx ** 2 + dy ** 2)
            accum_l += d
            if accum_l >= l:
                ind = i
                seg_l = d
                break

        ratio = 1. - (accum_l - l) / seg_l

        x = path[ind][0] + (path[ind + 1][0] - path[ind][0]) * ratio
        y = path[ind][1] + (path[ind + 1][1] - path[ind][1]) * ratio

        return [x, y, ind]

    @staticmethod
    def is_collision(first, second, obstacleList):
        x1, y1 = first[0], first[1]
        x2, y2 = second[0], second[1]
        mod_l = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        for (ox, oy, o_l) in obstacleList:
            vec_1 = ox - x1, oy - y1
            vec_2 = ox - x2, oy - y2
            d = abs(vec_1[0] * vec_2[1] - vec_1[1] * vec_2[0]) / mod_l
            if d <= o_l:
                return True
        return False

    @staticmethod
    def smoothing(path, max_iter, obstacle_list):
        """
        random two points a and b in the path, try to move from a to b directly.
        :param path:
        :param max_iter:
        :param obstacle_list:
        :return:
        """
        L = PathPostProcessor.calc_length(path)
        for i in range(max_iter):
            pick_points = [random.uniform(0, L), random.uniform(0, L)]
            pick_points.sort()

            first = PathPostProcessor.search_target_point(path, pick_points[0])
            second = PathPostProcessor.search_target_point(path, pick_points[1])
            if first[2] <= 0 or second[2] <= 0:  # first segment
                continue
            if second[2] + 1 >= len(path):  # last segment
                continue
            if first[2] == second[2]:  # in the same line.
                continue

            if is_collision(first, second, obstacle_list):
                continue
            newPath = []
            newPath.extend(path[:first[2]+1])
            newPath.append([first[0], first[1]])
            newPath.append([second[0], second[1]])
            newPath.extend(path[second[2] + 1:])
            path = newPath
            L = PathPostProcessor.calc_length(path)
            drawn_graph(path, obstacle_list)
        return path


def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[6, 10],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.planning()

    path = PathPostProcessor.smoothing(path, 1000, obstacleList)
    # Draw final path
    rrt.drawn_graph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
