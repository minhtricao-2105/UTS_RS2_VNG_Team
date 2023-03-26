#! /usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import math

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


def move_group_python_interface_tutorial():
  ## First initialize moveit_commander and rospy.
  print ("============ Starting tutorial setup")
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("ur3e_group")

  ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print ("============ Waiting for RVIZ...")
  rospy.sleep(0.1)

  ## Where the UR3E raw end effector is:
  print ("\n============ End Effector Location: ")
  pose=group.get_current_pose(end_effector_link="wrist_3_link").pose
  print (pose)
  print("\nWith Euler Angles:")
  print(euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
  
  ## Where the gripper end effector is (roughly cos it assumes a fixed poition, not the actual position):
  print ("\n============ End Effector Location: ")
  pose=group.get_current_pose(end_effector_link="rg2_l_finger_link").pose
  print (pose)
  print("\nWith Euler Angles:")
  print(euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
######ENDING PASTE

if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
