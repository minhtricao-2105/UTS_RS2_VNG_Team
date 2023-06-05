from function import*
from time import sleep
#Library Belongs to ROS:
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

import moveit_commander
import moveit_msgs.msg
import numpy as np
from function import*

#Library Belongs to Robot Toolbox:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 
from spatialmath import SE3
from math import pi
from math import degrees
import actionlib
import swift
import time
from ur3module import *
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal

#Library Belongs to Gripper
from onrobot_rg_control.msg import OnRobotRGOutput
from gripperFunction import *

# global varibale
joint_states = []
moveIt = False

# Funtion to subcribe to the joint state:
def joint_callback(msg):
    
    global joint_states
    
    joint_states = msg.position

#subcriber
rospy.init_node('robot_node')

sub1 = rospy.Subscriber('/joint_states', JointState, joint_callback)  

while moveIt == False:
    rospy.loginfo("Moving Robot to Home!!")
    moveit_commander.roscpp_initialize(sys.argv)
    joint_home_degree = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40]
    joint_home_radian = [math.radians(joint_home_degree) for joint_home_degree in joint_home_degree]
    arm = setUpRobot(0.5)
    arm.go(joint_home_radian)
    moveIt = True
    rospy.loginfo("Done Moving to Home!!")
    break

# Set up the client here
client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

goal = set_up_action_client()

robot = rtb.models.UR3() 
q_deg = [57.11, -106.32, 56.84, -44.7, -85.81, 328.40] # First position of the robot:
q_start = [x*pi/180 for x in q_deg]
robot.q = q_start

# Camera
cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) #relative pose with ee
cam_move(cam, robot, TCR)
camera_transform = cam.T #camera transform at capturing position

TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)
gripper_path = "/home/quangngo/Desktop/RTB-P Test files/SomeApplications/CAMGRIPPER.STL"
gripper = collisionObj.Mesh(filename=gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [0.5,0.1,0.1,1])
cam_move(gripper, robot, TGR)

# Define the point in Pixel frame
pixel_x_1 = 600
pixel_y_1 = 430
global_position_1 = cam_to_global(pixel_x_1,pixel_y_1, camera_transform) #global position
goal_obj_1 = collisionObj.Sphere(radius = 0.008, pose = SE3(global_position_1[0], global_position_1[1], global_position_1[2]),color = (0.5,0.1,0.1,1))
# env.add(goal_obj_1)

# Move to gripping point
path = move_to_pin(robot, q_start, global_position_1, turn = False)

pose_list = [robot.fkine(q) for q in path.q]

# Declare the step variable:
i = 0

for pose in pose_list:
    p = collisionObj.Sphere(radius = 0.005, pose = pose * SE3(0.2,0,0),color = (0.5,0.1,0.1,1))


dt = 10

for pose in pose_list:
    
    print("Step:", i)

    # print(joint_states)
    while np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose.A[0:3,3]) > 0.1:
        v, arrived = rtb.p_servo(robot.fkine(robot.q), pose, 1, threshold=0.1)
        vj = np.linalg.pinv(robot.jacobe(robot.q)) @ v # This is sending the velocity
        vj = vj.tolist()
        joint_step = [dt*x for x in vj]
        q = [x+y for x,y in zip(joint_states, joint_step)]

        # Clear the previous goal trajectory
        goal.trajectory.points.clear()

        # Sending command:
        point = JointTrajectoryPoint()

        point.positions = robot.q.tolist()

        print(point.positions)

        current_time = rospy.Time.now()

        point.time_from_start = current_time
        
        goal.trajectory.points.append(point)

        client.send_goal(goal)

        rospy.sleep(2)

        cam_move(cam, robot, TCR)

        cam_move(gripper, robot, TGR)
        
        # robot.q = robot.q + robot.qd * 0.05
        while not rospy.is_shutdown():
            print("Joint State: ", joint_states)
            break

        robot.q = joint_states
        # break
    i+=1

print("FINAL ERROR: ", np.linalg.norm(robot.fkine(robot.q).A[0:3,3] - pose_list[-1].A[0:3,3]))

# for q in path.q:
    
    # # goal.trajectory.points.clear()

    # goal = set_up_action_client()
    
    # point = JointTrajectoryPoint()

    # point.positions = q.tolist()

    # point.time_from_start = rospy.Time.now() + rospy.Duration.from_sec(0.2)
    
    # goal.trajectory.points.append(point)

    # print(goal)

    # client.send_goal(goal)

    # print("HEREE")
    
    # client.wait_for_result()
    
    # # result = client.get_result()
    
    # # print(result)

    # rospy.sleep(1)

# env.hold()
rospy.spin()