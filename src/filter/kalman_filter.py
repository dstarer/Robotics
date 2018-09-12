#!env/bin/python
# -*- encoding:utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


def one_dimension():
    n_iter = 50
    sz = (n_iter, )
    x = -5
    z = np.random.normal(x, 0.1, size=sz)
    print z
    Q = 1e-5  # model process variance
    R = 0.1 ** 2 # observation variance
    x_est = 0.0
    p_est = 1.0
    k = 0    # kalman gain

    X_est = [x_est]
    P_est = [p_est]

    for i in range(n_iter):
        # prediction
        x_prd = x_est # Ax + Bu, A = 1 and B = 0
        p_prd = p_est + Q # A * P * A.T + Q
        # update
        k = p_prd / (p_prd + R) # P' * H.T  * (H * P' * H.T + R)^-1, H = 1
        x_est = x_prd + k * (z[i] - x_prd)
        p_est = (1 - k) * p_prd # (1 - K * H) * P'
        X_est.append(x_est)
        P_est.append(p_est)

    plt.figure()
    plt.plot(z, 'k+', label="noisy measurements")
    plt.plot(X_est, 'b-', label="estimate value")
    plt.axhline(x, color='g', label='true value')
    plt.plot(P_est, 'yo', label="estimate variance")
    plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Voltage')
    plt.show()

# one_dimension()


def weight_estimate():
    z = np.array([158.0, 164.2, 160.3, 159.9, 162.1, 164.6, 169.6, 167.4, 166.4, 171.0, 171.2, 172.6])
    A = 1
    B = 1
    u_k = 1
    Q = 0.5
    H = 1
    R= 10
    x_est = 160
    p_est = 1.0
    X_est = [x_est]
    for i, v in enumerate(z):
        x_prd = A * x_est + B * u_k # x' = A * x + B * u
        p_prd = A * p_est * A + Q  # p' = A * p * A.T + Q

        #
        k = p_prd * H / (H * p_prd * H + R) # p' * H / (H * p' * H.T + R)
        x_est = x_prd + k * (v - H*x_est) # x = x' + k * (z - H*x')
        p_est = (1 - k*H) * p_prd   # p = (1 - K * H.T) * p'
        X_est.append(x_est)

    plt.figure()
    plt.plot(z, "k+", label="noise measurements")
    plt.plot(X_est, "b-", label="estimator values")
    plt.legend()
    plt.xlabel("Iteration")
    plt.ylabel("Voltage")
    plt.show()

# weight_estimate()


def kalman_xy(x, P, measurement, R, motion=np.matrix('0. 0. 0.1 0.1').T, Q=np.matrix(np.eye(4))):
    return kalman(x, P, measurement, R, motion, Q, F=np.matrix('''
    1. 0. 1. 0;
    0. 1. 0. 1;
    0. 0. 1. 0;
    0. 0. 0. 1.'''), H=np.matrix('''
    1. 0. 0. 0;
    0. 1. 0. 0.'''))


def kalman(x, P, measurement, R, motion, Q, F, H):
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R
    K = P * H.T * S.I

    x = x + K * y
    I = np.matrix(np.eye(F.shape[0]))
    P = (I - K * H) * P

    x = F * x + motion
    P = F * P * F.T + Q

    return x, P


def two_dimension_kalman():
    x = np.matrix('0. 0. 0. 0.').T
    P = np.matrix(np.eye(4)) * 1000

    N = 20
    true_x = np.linspace(0, 10.0, N)
    true_y = true_x ** 2
    observed_x = true_x + 0.05 * np.random.random(N) * true_x
    observed_y = true_y + 0.05 * np.random.random(N) * true_y
    plt.plot(observed_x, observed_y, 'ro')
    plt.plot(true_x, true_y, 'b+')
    result = []
    R = 0.01 ** 2
    for meas in zip(observed_x, observed_y):
        x, P = kalman_xy(x, P, meas, R)
        result.append((x[:2]).tolist())

    kalman_x, kalman_y = zip(*result)
    plt.plot(kalman_x, kalman_y, 'g-')
    plt.show()

two_dimension_kalman()