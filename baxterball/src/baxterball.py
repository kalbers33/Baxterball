#!/usr/bin/env python  
import roslib
from baxter_interface import CameraController
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import sys
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper

left_arm = 0
right_arm = 0
right_gripper = 0

def CameraStart():
    left_hand_camera = CameraController('left_hand_camera')
    left_hand_camera.open()
    left_hand_camera.resolution = (1280,800)
    left_hand_camera.exposure = 20
    #left_hand_camera.gain = 0;

def offset_holding(gripper, offset):
    current = gripper.parameters()['holding_force']
    gripper.set_holding_force(current + offset)

def MoveInit():
    #Wait for the IK service to become available
    global left_arm
    global right_arm
    global right_gripper
    #Set up the right gripper
    right_gripper = baxter_gripper.Gripper('right')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    left_arm.set_planner_id('RRTConnectkConfigDefault')
    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)
    #Calibrate gripper
    print('Calibrating gripper...') 
    right_gripper.calibrate()
    rospy.sleep(2.0)


def MoveTo(Pos, Rot):
    global left_arm
    global right_arm
    global right_gripper
    goal = PoseStamped()
    goal.header.frame_id = "base"

    #x, y, and z position
    goal.pose.position.x = Pos[0]
    goal.pose.position.y = Pos[1]
    goal.pose.position.z = Pos[2]+.1
    print goal.pose.position
    #Orientation as a quaternion
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = -1.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0

    #Open the right gripper
    print('Close...')
    right_gripper.close(block=True)
    rospy.sleep(0.5)
    offset_holding(right_gripper, 100.0)


    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal)

    #Set the start state for the left arm
    right_arm.set_start_state_to_current_state()

    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    right_arm.execute(right_plan)

    #Open the right gripper
    print('Opening...')
    right_gripper.open(block=True)
    rospy.sleep(0.5)
    offset_holding(right_gripper, 100.0)


if __name__ == '__main__':
    print "begin"
    rospy.init_node('baxterball')
    CameraStart()
    MoveInit()

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    print "start"
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base', '/ar_marker_7', rospy.Time(0))
            print trans
            MoveTo(trans, rot)
            rospy.sleep(2.0)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()