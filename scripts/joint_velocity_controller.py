from dynamics_wrapper import kdl_interface

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt

import numpy as np
from scipy.linalg import pinv

from qpsolvers import solve_qp, solve_ls

def cartesian_plot(t, x_list):

    ## stack list
    x_list = np.hstack(x_list).T
    rows = 2
    cols = 3
    titles = ["x", "y", "z", "alpha", "beta", "gamma"]
    ylim_upper = [1, 1, 1, 4, 4, 4]
    ylim_lower = [-1, -1, -1, -4, -4, -4]

    fig, axs = plt.subplots(rows, cols)

    k = 0
    for i in range(rows):
        for j in range(cols):
            axs[i, j].plot(t[:], x_list[:, k])
            axs[i, j].set_title(titles[k])
            axs[i, j].set_ylim([ylim_lower[k], ylim_upper[k]])
            k = k + 1
    
    plt.show()

def joint_plot(t, q_list):
    
    ## stack list
    q_list = np.hstack(q_list).T
    rows = 7
    cols = 1
    titles = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]

    fig, axs = plt.subplots(rows, cols)

    k = 0
    for i in range(rows):
        axs[i].plot(t[:], q_list[:, k])
        axs[i].set_title(titles[k])
        y_max = max(q_list[:, k]) + 0.2
        y_min = min(q_list[:, k]) - 0.2
        axs[i].set_ylim([y_min, y_max])
        k = k + 1

    plt.show()

def joint_limits_plot(t, q_list1, q_list2):
    
    ## stack list
    q_list1 = np.hstack(q_list1).T
    q_list2 = np.hstack(q_list2).T
    rows = 7
    cols = 1
    titles = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]

    fig, axs = plt.subplots(rows, cols)

    k = 0

    for i in range(rows):
        axs[i].plot(t[:], q_list1[:, k])
        axs[i].plot(t[:], q_list2[:, k])
        axs[i].set_title(titles[k])
        y_min = min(q_list1[:, k]) - 0.2
        y_max = max(q_list2[:, k]) + 0.2
        axs[i].set_ylim([y_min, y_max])
        k = k + 1

    plt.show()

def cart_joint_task(interface, q_e, x, qd_prev):

    # determine jacobian
    jacobian = interface.jac_geometric(q_e)
    vd = x
    
    # compute regularization in terms of tikhonov
    a1 = 0
    U1 = a1 * np.identity(7)
    u1 = a1 * np.zeros((7,))
    
    # compute regularization in terms of previous joint velocity
    a2 = 1
    U2 = a2 * np.identity(7)
    u2 = a2 * qd_prev

    # compute part of objective matrix and vector
    A = np.vstack((jacobian, U1 + U2))
    b = np.hstack((vd, u1 + u2))

    # compute resulting objective matrix and vector
    H = A.T @ A
    g = -A.T @ b

    return H, g


def limits(qd_p):

    # constants
    dt = 0.001
    qd_lim=np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.610, 2.610, 2.610])
    qdd_lim=np.array([15., 7.5, 10., 12.5, 15., 20., 20.])

    # check if same length
    assert len(qd_p) == len(qd_lim)

    # calculate upper and lower bound
    lb = qd_p - qdd_lim * dt
    ub = qd_p + qdd_lim * dt

    # run loop for both, should be a lambda functor
    for i in range(len(qd_p)):

        # ubber
        if abs(ub[i]) > qd_lim[i] and ub[i] > 0:
            ub[i] = qd_lim[i]
        elif abs(ub[i]) > qd_lim[i] and ub[i] < 0:
            ub[i] = -qd_lim[i] + qdd_lim * dt

        # lower
        if abs(lb[i]) > qd_lim[i] and lb[i] > 0:
            lb[i] = qd_lim[i] - qdd_lim * dt
        elif abs(lb[i]) > qd_lim[i] and lb[i] < 0:
            lb[i] = -qd_lim[i]

    return lb, ub



def controller_qp(interface, q_e, q_d, iterations):

    x_list = []
    q_list = []
    qlb_list = []
    qup_list = []
    dt = 0.001
    qd_p = np.zeros((7,))

    K = np.diag(np.array([1, 1, 1, 0.2, 0.2, 0.2]))

    for i in range(iterations):

        # calculate desired x_velocity
        xd = interface.car_error_quaternion(q_e, K @ q_d)
        
        # objectives
        H, g = cart_joint_task(interface, q_e, xd, qd_p)

        # determine limits
        # lb, ub = limits(qd_p)

        # calculate desired joint velocity
        qd_p = solve_qp(H, g, solver='osqp')
        
        # update desired position
        q_e.data = q_e.data + dt * qd_p

        # everything underneath here is for plotting
        
        x = interface.car_euler(q_e)
    
        x_list.append(np.expand_dims(x, axis=1))
        q_list.append(np.expand_dims(qd_p, axis=1))
        #qlb_list.append(np.expand_dims(lb, axis=1))
        #qup_list.append(np.expand_dims(ub, axis=1))
    
    t = np.arange(0, len(x_list)) * dt
    
    cartesian_plot(t, x_list)
    joint_plot(t, q_list)
    #joint_limits_plot(t, qlb_list, qup_list)



def main():

    # Input parameters to the serial link chain in KDL

    start = String("panda_link0")
    stop = String("panda_link8")

    # Create interface
    
    interface = kdl_interface("kdl_test")

    # Init KDL


    if not interface.initialize(start, stop).data:
        exit("KDL was not initialized")

    q_e = Float64MultiArray(data=[1, 1.3, 1.0, -1.5707, 0, 1.5707, 0])
    q_d = Float64MultiArray(data=[0, 0, 0, -2.0000, 0, 1.0000, 0])

    #controller(interface, q_e, q_d, 1000)
    controller_qp(interface, q_e, q_d, 1000)


if __name__ == "__main__":
    main()
