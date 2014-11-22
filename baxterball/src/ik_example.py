#!/usr/bin/env python

#MUST BE RUN WITH THE COMMAND LINE ARG /joint_states:=/robot/joint_states

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper

def main():
    #Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')

    #Create a goal pose for the left arm
    left_goal_pose = PoseStamped()
    left_goal_pose.header.frame_id = "base"

    #x, y, and z position
    left_goal_pose.pose.position.x = 0.5
    left_goal_pose.pose.position.y = 0.5
    left_goal_pose.pose.position.z = 0.3
    
    #Orientation as a quaternion
    left_goal_pose.pose.orientation.x = 0.0
    left_goal_pose.pose.orientation.y = -1.0
    left_goal_pose.pose.orientation.z = 0.0
    left_goal_pose.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    left_arm.set_pose_target(left_goal_pose)

    #Set the start state for the left arm
    left_arm.set_start_state_to_current_state()

    #Plan a path
    left_plan = left_arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the left arm: ')
    left_arm.execute(left_plan)

    #Move the right arm
    right_goal_pose = PoseStamped()
    left_goal_pose.header.frame_id = "base"
    #...

if __name__ == '__main__':
    main()