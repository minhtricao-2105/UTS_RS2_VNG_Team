#! /usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from onrobot_rg_control.msg import OnRobotRGOutput
import math
import numpy as np


from geometry_msgs.msg import *
from tf.transformations import *


def matrix_from_point_msg(point):
    return translation_matrix((point.x, point.y, point.z))


def matrix_from_quaternion_msg(quaternion):
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    return quaternion_matrix(q)


def matrix_from_pose_msg(pose):
    t = matrix_from_point_msg(pose.position)
    r = matrix_from_quaternion_msg(pose.orientation)
    return concatenate_matrices(t, r)


def point_msg_from_matrix(transformation):
    msg = Point()
    msg.x = transformation[0][3]
    msg.y = transformation[1][3]
    msg.z = transformation[2][3]
    return msg


def quaternion_msg_from_matrix(transformation):
    q = quaternion_from_matrix(transformation)
    msg = Quaternion()
    msg.x = q[0]
    msg.y = q[1]
    msg.z = q[2]
    msg.w = q[3]
    return msg


def pose_msg_from_matrix(transformation):
    msg = Pose()
    msg.position = point_msg_from_matrix(transformation)
    msg.orientation = quaternion_msg_from_matrix(transformation)
    return msg


def translate_pose_msg(pose, x, y, z):
    initial = matrix_from_pose_msg(pose)
    transform = translation_matrix((x, y, z))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_by_euler_angles(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(initial, transform))


def rotate_pose_msg_about_origin(pose, r, p, y):
    initial = matrix_from_pose_msg(pose)
    transform = quaternion_matrix(quaternion_from_euler(r, p, y))
    return pose_msg_from_matrix(concatenate_matrices(transform, initial))


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)
    return X, Y, Z


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def jointsAndEndEffectorInformation():
    # Where the UR3E raw end effector is:
    print("\n============ End Effector Location of Wrist: ")
    pose = group.get_current_pose(end_effector_link="wrist_3_link").pose
    print(pose)
    print("\nWith Euler Angles:")
    print(euler_from_quaternion(pose.orientation.x,
          pose.orientation.y, pose.orientation.z, pose.orientation.w))

    # Where the gripper end effector is (roughly cos it assumes a fixed poition, not the actual position):
    print("\n============ End Effector Location of Left Finger Link: ")
    pose = group.get_current_pose(end_effector_link="rg2_l_finger_link").pose
    print(pose)
    print("\nWith Euler Angles:")
    print(euler_from_quaternion(pose.orientation.x,
          pose.orientation.y, pose.orientation.z, pose.orientation.w))

    joint_states_vector = group.get_joint_value_target()
    for i in joint_states_vector:
        # print("Joint "+ str(i) +" is at: "+ str(int(joint_states_vector[i])) + " radians")
        print("Joint " + str(i) + " is at: " + str(int(i)) + " radians")


def genCommand(char, command):
    max_force = 400
    max_width = 1100
    try:
        command.rGFR = 400
        command.rGWD = min(max_width, int(char))
        command.rCTR = 16
    except ValueError:
        pass
    return command


def moveGripper(position):  # enter position when callling function
    position = int(position)

    command = OnRobotRGOutput()
    command = genCommand(position, command)
    pubGripper.publish(command)
    rospy.sleep(0.01)
    print("\n\n\nMoving Gripper To Position:" + str(position) + "\n\n")


# desiredSetPosition = either home OR zeros
def movingRobotToSetPoses(desiredSetPosition):
    print("============ Going to '" + desiredSetPosition + "' pose\n")
    group.set_named_target(desiredSetPosition)

    # Now, we call the planner to compute the plan and visualize it if successful
    plan1 = group.plan()
    rospy.sleep(0.1)
    # To move execute in moveit, and therefore move in Gazebo
    group.go(wait=True)


# Joint being changed is from 0-5, and desiredJointPosition is in radians
def changeSpecificJoint(joint_being_changed, desiredJointPosition):
    joint_states_vector = group.get_joint_value_target()

    print("changing joint " + str(joint_being_changed) + " which is currently at: " +
          str(joint_states_vector[joint_being_changed]) + " to: " + str(desiredJointPosition))
    joint_states_vector[joint_being_changed] = desiredJointPosition
    group.set_joint_value_target(joint_states_vector)
    group.go(wait=True)


# all 6 entered variables are the desired joint position values in radians
def changeAllJoints(dj0, dj1, dj2, dj3, dj4, dj5):
    joint_states_vector = group.get_joint_value_target()
    print("Original Joints:")
    for i in joint_states_vector:
        # print(str(joint_states_vector[i]))
        print(str(i))

    joint_states_vector = [dj0, dj1, dj2, dj3, dj4, dj5]
    print("Change Joints to:")
    for i in joint_states_vector:
        # print(str(joint_states_vector[i]))
        print(str(i))

    group.set_joint_value_target(joint_states_vector)
    group.go(wait=True)


# RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
def setEndEffectorWithOrientation(x, y, z, roll, pitch, yaw):
    pose_target = geometry_msgs.msg.Pose()

    q = get_quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    group.set_pose_target(pose_target)
    group.go(wait=True)


def cartesianPathPlanning():
    waypoints = []
    scale = 1
    pose = group.get_current_pose().pose
    pose.position.x -= scale * -0.2
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += scale * -0.3
    waypoints.append(copy.deepcopy(pose))
    pose.position.z -= scale * -0.1
    pose.position.y -= scale * -1.5
    waypoints.append(copy.deepcopy(pose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as
# the eef_step in Cartesian translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient for most cases
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # eef_step
        0.0)         # jump_threshold
    print("\n\nWaypoints:")
    print(waypoints)

    print("Executing:")
    group.execute(plan, wait=True)


# RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
def setEndEffectorWithOrientationNew(x, y, z, roll, pitch, yaw):
    pose_target = geometry_msgs.msg.Pose()

    q = get_quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.translate_pose_msg(0, 0, 0)

    group.set_pose_target(pose_target)
    group.go(wait=True)


# RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
def testingTranslateFunction(x, y, z):

    pose_target = geometry_msgs.msg.Pose()
    poseMatrix = translate_pose_msg(pose_target, x, y, z)

    pose_target.position.x = poseMatrix[0][3]
    pose_target.position.y = poseMatrix[1][3]
    pose_target.position.z = poseMatrix[2][3]

    group.set_pose_target(pose_target)
    group.go(wait=True)


if __name__ == '__main__':
    try:

        # INITIALIZE GRIPPER:
        pubGripper = rospy.Publisher(
            'OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

        # INITIALIZE ROBOT AND MOVEIT:
        # First initialize moveit_commander and rospy.
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('gripperAndRobotPublisher', anonymous=True)
        # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
        robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
        group = moveit_commander.MoveGroupCommander("ur3e_group")
        group2 = moveit_commander.MoveGroupCommander("rg2_group")

        # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        group.set_max_velocity_scaling_factor(1)
        group.set_max_acceleration_scaling_factor(1)
        # Optional: (hasb't been tested so might ruin it)
        group.set_planning_time(10)
        group.set_num_planning_attempts(3)
        group.allow_replanning(1)

        # GETTINGCURRENT POSITION/END EFFECTOR INFORMATION
        # jointsAndEndEffectorInformation()
        # MOVING ROBOT TO INPUTTED SET POSITION
        # movingRobotToSetPoses("home")
        # movingRobotToSetPoses("zeros")

        # MOVING GRIPPER TO INPUTTED POSITION
        # moveGripper(0)
        # MOVING SINGLE JOINT ON UR3E
        # changeSpecificJoint(1,-1)
        # MOVING ROBOT ARM TO INPUTTED VECTOR
        # changeAllJoints(0.1,0,0,0.5,1,1.1)
        # changeAllJoints(0,0,0,0,0,1)
        # SET END EFFECTOR:
        # setEndEffectorWithOrientation(0.4,0.2,0.3,1.5707, -1.5707,1.5707)
        # PERFORM CARTESIAN PATH PLANNING:
        # cartesianPathPlanning()
        # setEndEffectorWithOrientationNew(0.4,0.25,0.3,1.5707, -1.5707,1.5707)
        testingTranslateFunction(0, 0.1, 0.2)

        # GETTINGCURRENT POSITION/END EFFECTOR INFORMATION
        # jointsAndEndEffectorInformation()

    except rospy.ROSInterruptException:
        pass
