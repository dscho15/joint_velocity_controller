import numpy as np
import matplotlib.pyplot as plt


def cartesian_plot(t, x):

    rows = 2
    cols = 3

    x = np.hstack(x).T
    titles = ["x", "y", "z", "alpha", "beta", "gamma"]
    ylim_upper = [1, 1, 1, 4, 4, 4]
    ylim_lower = [-1, -1, -1, -4, -4, -4]

    fig, axs = plt.subplots(rows, cols)

    k = 0
    for i in range(rows):
        for j in range(cols):
            axs[i, j].plot(t[:], x[:, k])
            axs[i, j].set_title(titles[k])
            axs[i, j].set_ylim([ylim_lower[k], ylim_upper[k]])
            k = k + 1

    plt.show()


def joint_plot(t, q):

    rows = 7
    cols = 1

    titles = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]

    fig, axs = plt.subplots(rows, cols)

    k = 0
    for i in range(rows):
        axs[i].plot(t[:], q[:, k])
        axs[i].set_title(titles[k])
        y_max = max(q[:, k]) + 0.2
        y_min = min(q[:, k]) - 0.2
        axs[i].set_ylim([y_min, y_max])
        k = k + 1

    plt.show()


def joint_limits_plot(t, q_lower, q_upper):

    rows = 7
    cols = 1

    q_lower = np.hstack(q_lower).T
    q_upper = np.hstack(q_upper).T
    titles = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]

    fig, axs = plt.subplots(rows, cols)

    k = 0
    for i in range(rows):
        axs[i].plot(t[:], q_lower[:, k])
        axs[i].plot(t[:], q_upper[:, k])
        axs[i].set_title(titles[k])
        y_min = min(q_lower[:, k]) - 0.2
        y_max = max(q_upper[:, k]) + 0.2
        axs[i].set_ylim([y_min, y_max])
        k = k + 1

    plt.show()

def trajectory_plot(q):

    rows = 2
    cols = 1

    q = np.hstack(q).T[:,7:14]
    t = np.arange(0, len(q)) * 0.001
    joint_plot(t, q)
    


