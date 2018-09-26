#!venv/bin/python
# -*-encoding:utf-8-*-
import math
import matplotlib.pyplot as plt
import random


class Vertex(object):
    def __init__(self, x, y, parent=None, cost=0):
        self.x = x
        self.y = y
        self._parent = parent
        self.cost = 0

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, v):
        if isinstance(v, int):
            self._parent = v
            return
        raise ValueError("expected int")


class RRTStar(object):
    """
    RRT_star(X_init, expand_dist, K, obstacle_list):
        initial Graph G
        G.add_vertex(X_init)

        for k = 1 to K do:
            X_rand = RANDOM_STATE()
            search nearest neighbor X_nearest of X_rand in G.
            calculate the differences u between X_rand and X_nearest # control the car move from X_nearest to X_rand
            calculate the new state X_new = f(X_near, u, delta_t)
            S = query_near_points(G, X_new, d)
            parent, mincost = select_parent(S, X_new)
            X_new.parent = parent
            X_new.cost = mincost
            G.add_vertex(X_new)
            G.add_edge(parent, X_new, u);
            rectify(S, X_new)

        return get_final_course(goal)


    query_near_points(G, X_new, d):
        initial empty vertex set S
        for vertex in G.vertex():
            if distance(vertex, X_new) <= d:
                S.add(vertex)
        return S

    select_parent(S, X_new):
        parent = None
        mincost = INF
        for vertex in S:
            cost = vertex.cost + dist(X_new, vertex)
            if cost < mincost:
                cost = mincost
                parent = vertex
        return parent, mincost

    rectify(S, X_new):
        for vertex in S:
            cost = X_new.cost + dist(X_new, vertex)
            if cost < vertex.cost:
                vertex.cost = cost
                vertex.parent = X_new

    """

    def __init__(self, expand_dist=1.0, maxIter=200, goalSampleRate=20):
        self._expand_dist = expand_dist
        self.maxIter = maxIter
        self.G = []
        self.goalSampleRate = goalSampleRate

    def _clear(self):
        self.G = []

    def planning(self, outset, goal, obstacle_list, randArea):
        """
        :param outset:
        :param goal:
        :param obstacle_list:
        :return:
        """
        self._clear()
        self.G.append(Vertex(outset[0], outset[1]))
        for i in range(self.maxIter):
            point = self.get_rand_point(randArea, goal)
            ind = self.find_nearest_vertex(point)
            new_vertex = self.move_along_edge(ind, point)
            if self._is_collision(new_vertex, obstacle_list):
                continue
            near_inds = self.query_near_points(new_vertex)
            self.select_parent(near_inds, new_vertex, obstacle_list)
            self.G.append(new_vertex)
            self.rectify(near_inds, new_vertex, obstacle_list)
            drawn_graph(outset, goal, obstacle_list, self.G, point)
        return self.get_final_course(goal)

    def get_rand_point(self, randArea, goal):
        if random.randint(0, 100) > self.goalSampleRate:
            x = random.uniform(randArea[0], randArea[1])
            y = random.uniform(randArea[2], randArea[3])
            return [x, y]
        return goal

    def move_along_edge(self, begin_ind, end):
        nearest = self.G[begin_ind]
        theta = math.atan2(end[1] - nearest.y, end[0] - nearest.x)
        x = nearest.x + self._expand_dist * math.cos(theta)
        y = nearest.y + self._expand_dist * math.sin(theta)
        cost = nearest.cost + self._expand_dist
        new_vertex = Vertex(x, y, begin_ind, cost)
        return new_vertex

    def find_nearest_vertex(self, point):
        d = [(point[0] - v.x) ** 2 + (point[1] - v.y) ** 2 for v in self.G]
        return d.index(min(d))

    @staticmethod
    def _is_collision(vertex, obstacle_list):
        for (ox, oy, oz) in obstacle_list:
            if (ox - vertex.x) ** 2 + (oy - vertex.y) ** 2 <= oz * oz:
                return True
        return False

    def check_collision_line(self, vertex, theta, d, obstacle_list):
        p = Vertex(vertex.x, vertex.y)
        for i in range(int(d / self._expand_dist)):
            p.x += self._expand_dist * math.cos(theta)
            p.y += self._expand_dist * math.sin(theta)
            if self._is_collision(p, obstacle_list):
                return True
        return False

    def query_near_points(self, vertex):
        r = self._expand_dist * 10.0
        near_inds = []
        for i, point in enumerate(self.G):
            if (point.x - vertex.x) ** 2 + (point.y - vertex.y) ** 2 <= r ** 2:
                near_inds.append(i)
        return near_inds

    def select_parent(self, near_inds, vertex, obstacle_list):
        if len(near_inds) == 0:
            return vertex
        min_cost = vertex.cost
        min_ind = vertex.parent

        for ind in near_inds:
            p = self.G[ind]
            d = math.sqrt((p.x - vertex.x) ** 2 + (p.y - vertex.y) ** 2)
            cost = p.cost + d
            theta = math.atan2(vertex.y - p.y, vertex.x - p.x)
            if self.check_collision_line(p, theta, cost, obstacle_list):
                continue
            if cost <= min_cost:
                min_cost = cost
                min_ind = ind

        vertex.cost = min_cost
        vertex.parent = min_ind

    def rectify(self, near_inds, vertex, obstacle_list):
        id = len(self.G) - 1
        for ind in near_inds:
            p = self.G[ind]
            d = math.sqrt((p.x - vertex.x) ** 2 + (p.y - vertex.y) ** 2)
            theta = math.atan2(p.y - vertex.y, p.x - vertex.x)
            if self.check_collision_line(vertex, theta, d, obstacle_list):
                continue

            cost = vertex.cost + d
            if cost < p.cost:
                p.cost = cost
                p.parent = id

    def select_nearest_goal(self, goal):
        idx = -1
        min_cost = 1.e10
        for i, p in enumerate(self.G):
            d = math.sqrt((p.x - goal[0]) ** 2 + (p.y - goal[1]) ** 2)
            if d <= self._expand_dist:
                if idx == -1 or min_cost > p.cost + d:
                    idx = i
                    min_cost = p.cost + d
        return idx

    def get_final_course(self, goal):
        idx = self.select_nearest_goal(goal)
        if idx < 0 or idx >= len(self.G):
            return None
        path = [(goal[0], goal[1])]
        while idx:
            path.append((self.G[idx].x, self.G[idx].y))
            idx = self.G[idx].parent
        return path


def drawn_graph(start, goal, obstacle_list, G, rnd=None):
    plt.clf()
    if rnd is not None:
        plt.plot(rnd[0], rnd[1], "^k")
    for node in G:
        if node.parent is not None:
            plt.plot([node.x, G[node.parent].x], [node.y, G[node.parent].y], "-g")

    for (ox, oy, od) in obstacle_list:
        plot_circle(ox, oy, od)

    plt.plot(start[0], start[1], "xr")
    plt.plot(goal[0], goal[1], "xr")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.02)


def plot_circle(x, y, size):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    plt.plot(xl, yl, "-k")


def drawn(path, obstacleList):
    plt.clf()

    for (ox, oy, od) in obstacleList:
        plot_circle(ox, oy, od)

    plt.plot([x for (x, y) in path], [y for (x, y) in path], "-b")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.02)


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
    rrt = RRTStar()
    path = rrt.planning([0, 0], [6, 10], obstacleList, [-2, 15, -2, 15])
    if path is None:
        print "No path found"
        exit(0)
    import RRT
    processor = RRT.PathPostProcessor()
    aft_path = processor.smoothing(path, 100, obstacleList)
    # Draw final path
    drawn(path, obstacleList)
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.plot([x for (x, y) in aft_path], [y for (x, y) in aft_path], "-y")
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
