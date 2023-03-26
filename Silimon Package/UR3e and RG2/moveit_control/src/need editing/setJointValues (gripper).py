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
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
  group = moveit_commander.MoveGroupCommander("ur5_arm")
  #NEW:
  gripper= moveit_commander.MoveGroupCommander("gripper")

  ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(10)

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the robot.
  print "============ Printing robot state"
  print robot.get_current_state()


  ## Changing Joint States
  joint_being_changed=0
  desired = 0.5 #in radians
  joint_states_vector=group.get_joint_value_target()
  print(str(desired)) 
  print( "changing joint " + str(joint_being_changed) + " which is currently at: " + str(joint_states_vector[joint_being_changed]) + " to: " + str(desired) )
  joint_states_vector[joint_being_changed] =  desired
  group.set_joint_value_target(joint_states_vector)
  ##NEW:
  joint_being_changed=0
  desired = 0.7 #in radians
  print("Gripper info:")
  print(gripper.get_joint_value_target())
  
  joint_states_vector=gripper.get_joint_value_target()
  joint_states_vector[joint_being_changed] =  desired
  gripper.set_joint_value_target(joint_states_vector)  
  
  
  ## Now, we call the planner to compute the plan and visualize it if successful
  plan1 = group.plan()
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)
  ## To move execute in moveit, and therefore move in Gazebo
  group.go(wait=True)
  ##Now let's move the gripper:
  gripper.plan()
  print "============ Waiting while RVIZ moves gripper..."
  rospy.sleep(5)
  ## To move execute in moveit, and therefore move in Gazebo
  gripper.go(wait=True)


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
