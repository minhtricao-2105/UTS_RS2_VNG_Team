#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('scene_test', anonymous=True)

    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0.555 - (0.5 * 0.71)
    p.pose.position.z = -0.3 / 2 + 0.01
    p.pose.orientation.w = 1
    scene.add_box("table", p, (0.73, 0.71, 0.3))
    
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0.305
    p.pose.position.z = 0.011 / 2 + 0.01
    p.pose.orientation.w = 1
    scene.add_box("plate", p, (0.5, 0.32, 0.011))

if __name__ == "__main__":
    main()