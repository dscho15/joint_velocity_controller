from dynamics_wrapper import kdl_interface

import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt

from scipy.linalg import pinv

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


def controller(interface, q_e, q_d, iterations):

    x_list = []

    dt = 0.01

    for i in range(iterations):

        jac = interface.jac_geometric(q_d)

        jac_pinv = pinv(jac, rcond=1e-15)

        x_error = interface.car_error_quaternion(q_e, q_d)

        q_e.data = q_e.data + dt * jac_pinv @ x_error

        x = interface.car_euler(q_e)

        x_list.append(np.expand_dims(x, axis=1))

    t = np.arange(0, len(x_list))
    
    cartesian_plot(t, x_list)




def main():

    # Input parameters to the serial link chain in KDL

    start = String("panda_link0")
    stop = String("panda_link8")

    # Create interface
    
    interface = kdl_interface("kdl_test")

    # Init KDL

    if not interface.initialize(start, stop).data:
        exit("KDL was not initialized")

    q_e = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 1.5707, 0])
    q_d = Float64MultiArray(data=[0, 0, 0, -2.0000, 0, 1.0000, 0])

    controller(interface, q_e, q_d, 1000)


if __name__ == "__main__":
    main()
