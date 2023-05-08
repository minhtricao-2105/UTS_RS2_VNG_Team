import rospy
from sensor_msgs.msg import JointState
from math import degrees

def get_joint_positions_once():
    # Initialize a ROS node
    rospy.init_node("get_joint_positions_once")

    # Wait for the first joint state message to arrive
    msg = rospy.wait_for_message("/joint_states", JointState)

    # Extract the joint positions from the message
    joint_positions = list(msg.position)
    swap = joint_positions[0]
    joint_positions[0] = joint_positions[2]
    joint_positions[2] = swap
    degrees_list = [degrees(r) for r in joint_positions]
    print(degrees_list)



def joint_callback(msg):
    global current_joint_state
    current_joint_state = msg

if __name__ == '__main__':
    get_joint_positions_once()