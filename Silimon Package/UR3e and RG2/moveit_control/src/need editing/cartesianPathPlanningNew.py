#! /usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf.transformations as tf
import numpy as np


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

  #Optional:
  #group.set_planning_time(10)
  #group.set_num_planning_attempts(2)
  #group.allow_replanning(1)

  ## We can get a list of all the groups in the robot
  print ("============ Robot Groups:")
  print (robot.get_group_names())
  ## Sometimes for debugging it is useful to print the entire state of the robot.
  print ("============ Printing robot state")
  print (robot.get_current_state())

  ## Where the robot end effector is:
  print ("End Effector Location: ")
  print (group.get_current_pose(end_effector_link="wrist_3_link"))

  waypoints = []
  scale=1
  pose = group.get_current_pose().pose
  pose.position.x -= scale * 0.2
  waypoints.append(copy.deepcopy(pose))

  pose.position.z += scale * 0.3
  waypoints.append(copy.deepcopy(pose))

  pose.position.z -= scale * 0.1
  pose.position.y -= scale * 1.5
  waypoints.append(copy.deepcopy(pose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as
#the eef_step in Cartesian translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient for this tutorial.
  (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
  print("\n\nWaypoints:")
  print(waypoints)

  print("Executing:")
  group.execute(plan, wait=True)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print ("============ STOPPING")


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
