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
import math

def get_quaternion_from_euler(roll, pitch, yaw):

  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


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
  group = moveit_commander.MoveGroupCommander("ur5_manipulator")

  ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print ("============ Waiting for RVIZ...")
  rospy.sleep(10)

  #Optional:
  group.set_planning_time(10)
  group.set_num_planning_attempts(3)
  group.allow_replanning(1)

  ## We can get a list of all the groups in the robot
  print ("============ Robot Groups:")
  print (robot.get_group_names())
  ## Sometimes for debugging it is useful to print the entire state of the robot.
  print ("============ Printing robot state")
  print (robot.get_current_state())

  ## Where the robot end effector is:
  print ("End Effector Location: ")
  print (group.get_current_pose(end_effector_link="wrist_3_link"))

  ## Planning to a Pose goal
  pose_target = geometry_msgs.msg.Pose()

  # RPY to convert: 90deg, 0, -90deg
  q=get_quaternion_from_euler(1.5707, -1.5707,1.5707)

  print("\nDesired orientation:")
  print("\nWith quaternion format:")
  print(q[0],q[1],q[2],q[3])
  print("With Euler Angle format:")
  print(euler_from_quaternion(q[0],q[1],q[2],q[3]))
  
  pose_target.orientation.x = q[0]
  pose_target.orientation.y = q[1]
  pose_target.orientation.z = q[2]
  pose_target.orientation.w = q[3]

  pose_target.position.x = 0.15
  pose_target.position.y = 0.3
  pose_target.position.z = 0.3
  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan and visualize it if successful
  plan1 = group.plan()
  print ("============ Waiting while RVIZ displays plan1...")
  rospy.sleep(5)
  # To move execute in moveit, and therefore move in Gazebo
  group.go(wait=True)

  ## Where the robot end effector is using quaternion using current pose information:
  pose=group.get_current_pose().pose
  print("\nPrinting Pose:")
  print (pose)
  print("Note: orientation this may be different values, but it should be very similar/same as it can rotate different ways")
  print("\nCurrent orientation using pose information with Euler Angle format:")
  print(euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print ("============ STOPPING")


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
