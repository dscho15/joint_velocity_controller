from dynamics_wrapper import kdl_interface

from gazebo_msgs.msg import LinkStates
from trajectory_msgs.msg import JointTrajectory

from std_msgs.msg import String

import matplotlib.pyplot as plt

import numpy as np
import rospy

from scipy.linalg import pinv

from qpsolvers import solve_qp

handle = None
joint_pos = None


def handle_callback(msg):
    print("hello")


def robot_callback(msg):
    print("hello")


def main():

    # Input parameters to the serial link chain in KDL
    start = String("panda_link0")
    stop = String("panda_link8")

    # Init Node
    rospy.init_node('controller_qp', anonymous=True)

    # Subscribe to the state of the joint impedance controller - [q/qd]
    sub_handle = rospy.Subscriber(
        '/gazebo/link_states', LinkStates, handle_callback)

    # Subscribe to the state of the tool rotation matrix
    sub_robot_rotm = rospy.Subscriber(
        '/joint_position_setpoint_controller/panda/tool_rotation_matrix', JointTrajectory, robot_callback)

    # Subscribe to the state of the joint impedance controller
    sub_robot = rospy.Subscriber(
        '/joint_position_setpoint_controller/panda/joint_state', JointTrajectory, robot_callback)

    # Subscribe to the publisher of the joint impedance controller
    pub = rospy.Publisher(
        '/joint_position_setpoint_controller/panda/move_robot', JointTrajectory, queue_size=1)

    # Rate update
    rate = rospy.Rate(1000)

    # Create interface
    interface = kdl_interface("kdl_test")

    # Intialize KDL interface
    if not interface.initialize(start, stop).data:
        exit("KDL was not initialized")


if __name__ == "__main__":
    main()
