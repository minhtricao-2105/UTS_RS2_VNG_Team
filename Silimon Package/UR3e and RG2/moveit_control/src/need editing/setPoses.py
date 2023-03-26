#! /usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def move_group_python_interface_tutorial():

  ## First initialize moveit_commander and rospy.
  print( "============ Starting tutorial setup")
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("ur5_manipulator")

  ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print ("============ Waiting for RVIZ...")
  rospy.sleep(10)


  ## We can get a list of all the groups in the robot
  print ("============ Robot Groups:")
  print (robot.get_group_names())
  ## Sometimes for debugging it is useful to print the entire state of the robot.
  print ("============ Printing robot state")
  print (robot.get_current_state())


  ## Planning to a Pose goal
  ## We can plan a motion for this group to a desired pose for the end-effector
  print ("============ Going to 'allZeroes' pose")
  group.set_named_target("allZeroes")
  #print ("============ Going to 'random' pose")
  #group.set_named_target("random")
  #print ("============ Going to 'diagonal' pose")
  #group.set_named_target("diagonal")


  ## Now, we call the planner to compute the plan and visualize it if successful
  plan1 = group.plan()
  print ("============ Waiting while RVIZ displays plan1...")
  rospy.sleep(5)
  ## To move execute in moveit, and therefore move in Gazebo
  group.go(wait=True)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print ("============ STOPPING")


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
