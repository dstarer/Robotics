#!venv/bin/python
# -*-encoding:utf-8-*-

import sys
sys.path.append("../PathPlanning/")
import numpy as np
import math
from car import Car
import cvxpy
import matplotlib.pyplot as plt
import cubic_spline_planner


# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 2.5

class Constants(object):
    NUM_STATE = 4
    NUM_CONTROL = 2
    NUM_PREDICTION = 5
    CAR_LENGTH = 4.5
    CAR_WIDTH = 2.0
    BACKTOWHEEL = 1.0
    WHEEL_LEN = 0.3
    WHEEL_WIDTH = 0.2
    TREAD = 0.7
    WB = 2.5
    MAX_STEER = math.radians(45.0)
    MAX_DSTEER = math.radians(30.0)
    MAX_SPEED = 55.0 / 3.6
    MIN_SPEED = -20.0 / 3.6
    MAX_ACCEL = 1.0
    DT = 0.2
    N_IND_SEARCH = 10
    T = 5
    GOAL_DIS = 1.5
    STOP_SPEED = 0.5 / 3.6
    SHOW_ANIMATION = True
    TARGET_SPEED = 10.0 / 3.6
    MAX_TIME = 500.0


def get_linear_model_matrix(v, phi, delta):
    A = np.matrix(np.zeros((Constants.NUM_STATE, Constants.NUM_STATE)))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = Constants.DT * math.cos(phi)
    A[0, 3] = - Constants.DT * v * math.sin(phi)
    A[1, 2] = Constants.DT * math.sin(phi)
    A[1, 3] = Constants.DT * v * math.cos(phi)
    A[3, 2] = Constants.DT * math.tan(delta) / Constants.WB

    B = np.matrix(np.zeros((Constants.NUM_STATE, Constants.NUM_CONTROL)))
    B[2, 0] = Constants.DT
    B[3, 1] = Constants.DT  * v / (Constants.WB * math.cos(delta) ** 2)

    C = np.zeros(Constants.NUM_STATE)
    C[0] = Constants.DT * v * math.sin(phi) * phi
    C[1] = - Constants.DT * v * math.cos(phi) * phi
    C[3] = v * delta / (Constants.WB * math.cos(delta) ** 2)

    return A, B, C


def get_narray_from_matrix(x):
    return np.array(x).flatten()


def pi_2_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None

    def move(self, a, delta):
        if delta >= Constants.MAX_STEER:
            delta = Constants.MAX_STEER
        elif delta <= -Constants.MAX_STEER:
            delta = -Constants.MAX_STEER

        self.x = self.x + self.v * math.cos(self.yaw) * Constants.DT
        self.y = self.y + self.v * math.sin(self.yaw) * Constants.DT
        self.yaw = self.yaw + self.v / Constants.WB * math.tan(delta) * Constants.DT
        self.v = self.v + a * Constants.DT

        if self.v > Constants.MAX_SPEED:
            self.v = Constants.MAX_SPEED
        elif self.v < Constants.MIN_SPEED:
            self.v = Constants.MIN_SPEED

        return [self.x, self.y, self.yaw, self.v]


class MPC:
    def __init__(self, R, Rd, Q, Qf, T = 5, goal_dist = 1.5, stop_speed=0.5/3.6, max_iter=3, du_th=0.1):
        self.R = R
        self.Rd = Rd
        self.Q = Q
        self.Qf = Qf
        self.GOAL_DIST = goal_dist
        self.STOP_SPEED = stop_speed
        self.MAX_ITER = max_iter
        self.DU_TH = du_th
        self.DT = 0.2
        self.T = T

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        :param xref: reference line (sampled) , np.shape(xref) = (T, len(x0))
        :param x0: state vector
        :param dref:
        :param oa: the initial ans for control variable acc [random.value()] * np.shape(xref)[1]
        :param od: the initial ans for control variable delta [random.value] * np.shape(xref)[1]
        :return:
            oa: a group control variable acc
            od: a group control variable delta
            ox: prediction state x
            oy: prediction state y
            oyaw: prediction state yaw
            ov: prediction state v
        """
        if oa is None or od is None:
            oa = [0.0] * self.T
            od = [0.0] * self.T

        for i in range(self.MAX_ITER): # iterate to get the best answer
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od, ox, oy, oyaw, ov = self.linear_mpc_control(xref, xbar, x0, dref)
            du = sum(abs(oa -poa)) + sum(abs(od - pod))
            if du < self.DU_TH:
                break
        else:
            print("Iterative is max iter")

        return oa, od, ox, oy, oyaw, ov

    def predict_motion(self, x0, oa, od, xref):
        """
        predict a series states according to initial state x0, and a series actions of oa, od
        :param x0: initial state m * 1
        :param oa: probably action accel 1 * (T - 1)
        :param od: probably action delta 1 * (T - 1)
        :param xref: the reference line m * n
        :return:
        """
        xbar = xref * 0.0
        for i in range(len(x0)):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state.move(ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar

    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control
        :param xref: reference point
        :param xbar: operational point
        :param x0: initial state
        :param dref: reference steer angle
        :return:
        """
        x = cvxpy.Variable((Constants.NUM_STATE, self.T + 1))
        u = cvxpy.Variable((Constants.NUM_CONTROL, self.T))

        cost = 0.0
        constraints = []
        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)
            A, B, C = get_linear_model_matrix(xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= Constants.MAX_DSTEER * self.DT]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)

        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= Constants.MAX_SPEED]
        constraints += [x[2, :] >= Constants.MIN_SPEED]
        constraints += [cvxpy.abs(u[0, :]) <= Constants.MAX_ACCEL]
        constraints += [cvxpy.abs(u[1, :]) <= Constants.MAX_STEER]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            ox = get_narray_from_matrix(x.value[0, :])
            oy = get_narray_from_matrix(x.value[1, :])
            ov = get_narray_from_matrix(x.value[2, :])
            oyaw = get_narray_from_matrix(x.value[3, :])
            oa = get_narray_from_matrix(u.value[0, :])
            od = get_narray_from_matrix(u.value[1, :])
        else:
            print("Error: Cannot solve mpc ...")
            oa, od, ox, oy, oyaw, ov =None, None, None, None, None, None

        return oa, od, ox, oy, oyaw, ov


def calc_nearest_index(state, cx, cy, cyaw, pind):
    """
    :param state: current  car's state (x, y, v, yaw)
    :param cx: waypoints'x
    :param cy: waypoints'y
    :param cyaw: waypoints'yaw
    :param pind: search range [pind, N] n denotes the number of waypoints
    :return:
    """
    dx = [state.x - icx for icx in cx[pind: (pind + Constants.N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind: (pind + Constants.N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    return ind, mind


def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, pind):
    xref = np.zeros((Constants.NUM_STATE, Constants.T + 1))
    dref = np.zeros((1, Constants.T + 1))

    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)
    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0 # steer operational point should be 0
    travel = 0.0

    for i in range(Constants.T + 1):
        travel += abs(state.v) * Constants.DT
        dind = int(round(travel/dl))
        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0
    return xref, ind, dref


def check_goal(state, goal, tind, nind):
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    if d <= Constants.GOAL_DIS:
        is_goal = True
    else:
        is_goal = False

    if abs(tind - nind) >= 5:
        is_goal = False

    if abs(state.v) <= Constants.STOP_SPEED:
        is_stop = True
    else:
        is_stop = False

    if is_goal and is_stop:
        return True

    return False


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0

    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds = dl)
    return cx, cy, cyaw, ck


def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck


def get_straight_course3(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl):
    ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):

    outline = np.matrix([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.matrix([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.matrix([[math.cos(yaw), math.sin(yaw)],
                      [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.matrix([[math.cos(steer), math.sin(steer)],
                      [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T * Rot2).T
    fl_wheel = (fl_wheel.T * Rot2).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T * Rot1).T
    fl_wheel = (fl_wheel.T * Rot1).T

    outline = (outline.T * Rot1).T
    rr_wheel = (rr_wheel.T * Rot1).T
    rl_wheel = (rl_wheel.T * Rot1).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state, mpc):
    """
    Simulation
    :param cx: course x position list
    :param cy: course y position list
    :param cyaw: course yaw position list
    :param ck: course curvature list
    :param sp: speed profile
    :param dl: course tick [m]
    :param initial_state:
    :return:
    """
    goal = [cx[-1], cy[-1]]
    state = initial_state
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)
    odelta, oa = None, None
    cyaw = smooth_yaw(yaw)

    while Constants.MAX_TIME >= time:
        xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]

        oa, odelta, ox, oy, oyaw, ov = mpc.iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

        if odelta is not None:
            di, ai = odelta[0], oa[0]

        state.move(ai, di)
        time = time + Constants.DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        if check_goal(state, goal, target_ind, len(cx)):
            print "Goal"
            break

        if Constants.SHOW_ANIMATION:
            plt.cla()
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2)) + ", speed[km/h]: " + str(round(state.v * 3.6, 2)))
            plt.pause(0.0001)

    return t, x, y, yaw, v, d, a

import random

def main():
    print(__file__ + "start !!")
    dl = 1.0
    cx, cy, cyaw, ck = get_straight_course3(dl)

    sp = calc_speed_profile(cx, cy, cyaw, Constants.TARGET_SPEED)

    initial_state = State(random.gauss(cx[0], 1), y=random.gauss(cy[0], 1), yaw=random.gauss(cyaw[0], 0.2), v=0.0)
    R = np.diag([0.01, 0.01])
    Rd = np.diag([0.01, 1.0])
    Q = np.diag([1.0, 1.0, 0.5, 1.0])
    Qf = Q

    mpc = MPC(R, Rd, Q, Qf)

    t, x, y, yaw, v, d, a = do_simulation(cx, cy, cyaw, ck, sp, dl, initial_state, mpc)

    if Constants.SHOW_ANIMATION:
        plt.close("all")
        plt.subplots()
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()

        plt.subplots()
        plt.plot(t, v, "-r", label="speed")
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [kmh]")

        plt.show()


if __name__ == '__main__':
    main()
