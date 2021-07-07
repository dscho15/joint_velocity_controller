import rospy 
from gazebo_msgs.msg import LinkStates

def callback_linkstate(msg):
    try:
        array_idx = msg.name.index('panda::handle_link')
    except ValueError as e:
        # Wait for Gazebo to open
        pass
    else:
        print(msg.pose[array_idx])
        msg.twist[array_idx]

def main():
    rospy.init_node('get_state', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, callback_linkstate)
    rospy.spin()

try:
    main()
except rospy.ROSInterruptException:
    pass