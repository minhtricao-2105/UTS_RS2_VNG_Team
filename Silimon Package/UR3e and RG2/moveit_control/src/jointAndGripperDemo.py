#! /usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from onrobot_rg_control.msg import OnRobotRGOutput


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


def publisher():

    pubGripper = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    command = OnRobotRGOutput()
    position=300
    position=int(position)

    command = genCommand(position, command)
    pubGripper.publish(command)
    rospy.sleep(0.01)
    
    #Printing
    print ("moved to position:"+ str(position) )
    print ("============ STOPPING")


def move_group_python_interface_tutorial():
  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print ("============ Waiting for RVIZ...")
  rospy.sleep(0.1)

  ## We can get a list of all the groups in the robot
  print ("============ Robot Groups:")
  print (robot.get_group_names())
  ## Sometimes for debugging it is useful to print the entire state of the robot.
  print ("============ Printing robot state")
  print (robot.get_current_state())


  ## Changing Joint States
  joint_being_changed=3
  desired = -1 #in radians
  joint_states_vector=group.get_joint_value_target()
  print(str(desired)) 
  print( "changing joint " + str(joint_being_changed) + " which is currently at: " + str(joint_states_vector[joint_being_changed]) + " to: " + str(desired) )
  joint_states_vector[joint_being_changed] =  desired
  group.set_joint_value_target(joint_states_vector)
  
  
  ## Now, we call the planner to compute the plan and visualize it if successful
  plan1 = group.plan()
  print ("============ Waiting while RVIZ displays plan1...")
  rospy.sleep(0.1)

  ## To move execute in moveit, and therefore move in Gazebo
  group.go(wait=True)


if __name__=='__main__':
  try:
     
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('jointAndGripperDemo',anonymous=True)
    ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
    group = moveit_commander.MoveGroupCommander("ur3e_group")
    ## We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
    move_group_python_interface_tutorial()
    #MOVING GRIPPER
    # publisher()
    rospy.on_shutdown(publisher)
  except rospy.ROSInterruptException:
    pass
