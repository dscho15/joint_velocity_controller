from dynamics_wrapper import kdl_interface

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

import matplotlib.pyplot as plt

import numpy as np
import rospy
from plot_util import cartesian_plot, joint_limits_plot, joint_plot, trajectory_plot

from scipy.linalg import pinv

from qpsolvers import solve_qp

q_list = []
qe_list = []

def joint_callback(data):
    q_list.append(np.expand_dims((data.data), axis = 1))


def cart_joint_task(jac, x, qd_prev):

    # Determine jacobian
    jacobian = jac
    vd = x

    # Compute regularization in terms of tikhonov
    a1 = 0.5
    U1 = a1 * np.identity(7)
    u1 = a1 * np.zeros((7,))

    # Compute regularization in terms of previous joint velocity
    a2 = 0.5
    U2 = a2 * np.identity(7)
    u2 = a2 * qd_prev

    # Compute part of objective matrix and vector
    A = np.vstack((jacobian, 3*(U1 + U2)))
    b = np.hstack((vd, u1 + u2))

    # Compute resulting objective matrix and vector
    H = A.T @ A
    g = -A.T @ b

    return H, g


def limits(qd_p):

    # Constants
    dt = 0.001
    qd_lim = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.610, 2.610, 2.610])
    qdd_lim = np.array([15., 7.5, 10., 12.5, 15., 20., 20.])

    # Check if same length
    assert len(qd_p) == len(qd_lim)

    # Calculate upper and lower bound
    lb = qd_p - qdd_lim * dt
    ub = qd_p + qdd_lim * dt

    # Run loop for both, should be a lambda functor
    for i in range(len(qd_p)):

        # Upper limits
        if abs(ub[i]) > qd_lim[i] and ub[i] > 0:
            ub[i] = qd_lim[i]
        elif abs(ub[i]) > qd_lim[i] and ub[i] < 0:
            ub[i] = -qd_lim[i] + qdd_lim[i] * dt

        # Lower limits
        if abs(lb[i]) > qd_lim[i] and lb[i] > 0:
            lb[i] = qd_lim[i] - qdd_lim[i] * dt
        elif abs(lb[i]) > qd_lim[i] and lb[i] < 0:
            lb[i] = -qd_lim[i]

    return lb, ub


def main():

    # Input parameters to the serial link chain in KDL
    start = String("panda_link0")
    stop = String("panda_link8")

    # Init Node
    rospy.init_node('controller_qp', anonymous=True)

    # Subscribe to the state of the joint impedance controller - [q/qd]
    sub = rospy.Subscriber(
        '/franka_joint_impedance_controller/joint_state', Float64MultiArray, joint_callback)

    # Subscribe to the publisher of the joint impedance controller
    pub = rospy.Publisher(
        '/franka_joint_impedance_controller/command', Float64MultiArray, queue_size=1)

    # Rate update
    rate = rospy.Rate(1000)

    # Create interface
    interface = kdl_interface("kdl_test")

    # Intialize KDL interface
    if not interface.initialize(start, stop).data:
        exit("KDL was not initialized")

    # Start and stop position, should probably take the robot to the Start Pose before anything else
    q_e = Float64MultiArray(data=[0, 0, 0, -1.5707, 0, 1.5707, 0])
    
    # Place robot in position
    for i in range(10000):
        pub.publish(q_e)
        rate.sleep()

    # Set desired pose
    q_d = Float64MultiArray(data=[0, 0, 0, -2.0000, 0, 1.57000, 0])

    # Controller starts here
    it = 25000

    # Desired velocity should be read
    qd_p = np.zeros((7,))

    for i in range(it):

        # Calculate desired x_velocity
        xd = interface.pose_error_quaternion(q_e, q_d)

        # Determine gains
        xd = np.diag(np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0])) @ xd

        # Objectives
        H, g = cart_joint_task(interface.jac_geometric(q_e), xd, qd_p)

        # Determine limits
        qd_d = q_list[-1][7:14]
        lb, ub = limits(qd_d.squeeze(axis=1))

        # Calculate desired joint velocity
        qd_p = solve_qp(H, g, lb=lb, ub=ub, solver='osqp')

        # Update desired position
        q_e.data = q_e.data + 0.001 * qd_p

        # Publish desired positon
        pub.publish(q_e)

        # Sleep node
        rate.sleep()

        # Update qe
        qe_list.append(np.expand_dims((q_e.data), axis = 1))
    
    trajectory_plot(q_list[-25000:-1])

if __name__ == "__main__":
    main()
