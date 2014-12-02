#!/usr/bin/env python  
import roslib
from baxter_interface import CameraController
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import sys
import os as os
import moveit_commander
import numpy as np
import transformations as tftrans
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
from itertools import *

left_arm = 0
right_arm = 0
right_gripper = 0
left_hand_camera = 0
right_hand_camera = 0
listener = 0
Ball_list = []
br = 0

def CameraStart(Camera):
    Camera.open()
    Camera.resolution = (1280,800)
    Camera.exposure = 15
    Camera.gain = 20 #CameraController.CONTROL_AUTO;

def CameraInit():
    global left_hand_camera
    global right_hand_camera
    left_hand_camera = CameraController('left_hand_camera')
    left_hand_camera.close()
    right_hand_camera = CameraController('right_hand_camera')
    right_hand_camera.close()
    CameraStart(left_hand_camera)
    CameraClose(left_hand_camera)
    CameraStart(right_hand_camera)
    CameraClose(right_hand_camera)


def CameraOpen(Camera):
    Camera.open()

def CameraClose(Camera):
    Camera.close()


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


def MoveTo(Arm, Pos, rot = (0.0,-1.0,0.0,0.0), PosOffset = (0.0,0.0,0.0)):
    goal = PoseStamped()
    goal.header.frame_id = "base"

    #x, y, and z position
    goal.pose.position.x = Pos[0] + PosOffset[0]
    goal.pose.position.y = Pos[1] + PosOffset[1]
    goal.pose.position.z = Pos[2] + PosOffset[2]
    #print goal.pose.position
    #Orientation as a quaternion
    goal.pose.orientation.x = rot[0]
    goal.pose.orientation.y = rot[1]
    goal.pose.orientation.z = rot[2]
    goal.pose.orientation.w = rot[3]

    Arm.set_goal_orientation_tolerance = 0.5

    #Set the goal state to the pose you just defined
    Arm.set_pose_target(goal)

    #Set the start state for the left arm
    Arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    '''orien_const = OrientationConstraint()
    orien_const.link_name = "left_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 1.0;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    left_arm.set_path_constraints(consts)'''

    #print Arm.get_goal_position_tolerance()
    #print Arm.get_goal_orientation_tolerance()

    #Plan a path
    right_plan = Arm.plan()
    #Execute the plan
    Arm.execute(right_plan)

def GripperOpen():#Open the right gripper
    print('Gripper Opening...')
    right_gripper.open(block=True)
    rospy.sleep(0.5)
    offset_holding(right_gripper, 100.0)

def GripperClose():
    #Open the right gripper
    print('Gripper Closing...')
    right_gripper.close(block=True)
    rospy.sleep(0.5)
    offset_holding(right_gripper, 100.0)

def UpdateBallTransforms():
    global br
    global Ball_list
    br.sendTransform((0.08, 0.0, 0.0), #(0.065, 0.0, 0.0) OLD OFFSET
        (0.0,0.0,0.0,1.0), 
        rospy.Time.now(), 
        'Basket', 
        'ar_marker_8')
    if len(Ball_list) > 0:
        for t in Ball_list:
            br.sendTransform((0.065, 0.0, 0.025), #(0.065, 0.0, 0.015) OLD OFFSET
                (0.0,0.0,0.0,1.0), 
                rospy.Time.now(), 
                t[0][1:], 
                t[2][1:])

def LookupTransform(Source, End, Guess):
    global listener
    now = rospy.Time.now()
    for x in range(0,50):
        try:
            UpdateBallTransforms()
            return listener.lookupTransform(Source, End, rospy.Time(0))#(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.sleep(0.1)
            continue
    print "TF ERROR: Cannot Find Transform"
    return Guess

def FindZRotation(q):
    q2 = tftrans.quaternion_about_axis(np.pi, (0, 1.0, 0))
    return tftrans.quaternion_multiply(q, q2)

def FindBasket(InitialLocation, Marker, Basket, zUpOffset = 0.25):
    global right_arm
    #Move to last known location
    MoveTo(right_arm, InitialLocation[0], FindZRotation(InitialLocation[1]), (0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Correct Location
    (location,rot) = LookupTransform('/base', Marker,InitialLocation)
    MoveTo(right_arm, location, FindZRotation(rot), ( 0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Find location one last time and return this
    (location,rot) = LookupTransform('/base', Basket,(location,rot))
    return (location,rot)

def PickUpBall(InitialLocation, Marker, zUpOffset = 0.25):
    global right_arm
    #Go to originaly spotted location
    MoveTo(right_arm, InitialLocation[0], FindZRotation(InitialLocation[1]), (0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Adjust using the new camera data
    (location,rot) = LookupTransform('/base', Marker,InitialLocation)
    MoveTo(right_arm, location, FindZRotation(rot), (0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Adjust using the new camera data
    '''(location,rot) = LookupTransform('/base', Marker,(location,rot))
    MoveTo(right_arm, location, FindZRotation(rot), (0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)'''
    #Second Adjust
    (location,rot) = LookupTransform('/base', Marker,(location,rot))
    MoveTo(right_arm, location, FindZRotation(rot), (0.0, 0.0, zUpOffset/2))
    rospy.sleep(0.5)
    #Move down to pick up the ball
    (location,rot) = LookupTransform('/base', Marker,(location,rot))
    MoveTo(right_arm, location, FindZRotation(rot), (0.0, 0.0, 0.0))
    #Grip the ball and pick it up
    GripperClose()
    MoveTo(right_arm, location, FindZRotation(rot), (0.0, 0.0,  zUpOffset))

def DropBallInBasket(InitialLocation, Marker, zUpOffset = 0.18):
    global right_arm
    #Move to last known location
    MoveTo(right_arm, InitialLocation[0], FindZRotation(InitialLocation[1]), (0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Correct Location then drop the ball
    (location,rot) = LookupTransform('/base', Marker,InitialLocation)
    MoveTo(right_arm, location, FindZRotation(rot), ( 0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)
    #Correct Location then drop the ball
    '''(location,rot) = LookupTransform('/base', Marker,(location,rot))
    MoveTo(right_arm, location, FindZRotation(rot), ( 0.0, 0.0, zUpOffset))
    rospy.sleep(0.5)'''
    GripperOpen()
    (location,rot) = LookupTransform('/base', Marker,(location,rot))

if __name__ == '__main__':
    print "Begin Init"
    Ball_list = []
    #os.system("rosrun baxter_tools tuck_arms.py -t")
    rospy.init_node('baxterball')
    CameraInit()
    MoveInit()
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    print "Start Running"
    while not rospy.is_shutdown():
        #try:
        #Init Arm config
        GripperOpen()
        print 'Moving Left Arm'
        MoveTo(left_arm, (0.6, 0.4, .2), (0.70710678118,0.70710678118,0.0,0.0))
        CameraClose(left_hand_camera)
        CameraOpen(right_hand_camera)
        print "Opening Right Camera"
        '''try:
            MoveTo(left_arm, (0.70897, -0.16858, -0.055098), (0.73035, 0.53993, -0.32156, 0.2677))
        except(e):
            print "Error, Could not move left arm"'''
        print 'Moving Right Arm'
        try:
            MoveTo(right_arm, (0.6, -0.4, .2), (0.70710678118,0.70710678118,0.0,0.0))#(0.0, -1.0, 0.0, 0.0))
        except(e):
            print "Error, Could not move right arm"
        #print "Opening Left Camera"
        #CameraClose(right_hand_camera)
        #CameraOpen(left_hand_camera)
        print "Finding Basket"
        Basket_location = LookupTransform('/base', '/ar_marker_8', ((0.0,0.0,0.0),(0.0,0.0,0.0,1.0)))
        print "Basket Location: " + str(Basket_location)
        '''br.sendTransform((0.065, 0.0, 0.00), 
            (0.0,0.0,0.0,1.0), 
            rospy.Time.now(), 
            'Basket', 
            'ar_marker_8')'''
        Ball_location = ((0.0,0.0,0.0),(0.0,0.0,0.0,1.0))
        Ball_list = []
        for tag in [('ar_marker_2', 'Ball_1'),('ar_marker_1', 'Ball_2'),('ar_marker_5', 'Ball_3'),]:
            try:
                listener.waitForTransform('/base', '/'+ tag[0], rospy.Time(0), rospy.Duration(3.0))
                Ball_location = LookupTransform('/base', '/'+ tag[0], Ball_location)
                br.sendTransform((0.065, 0.0, 0.015), 
                    (0.0,0.0,0.0,1.0), 
                    rospy.Time.now(), 
                    tag[1], 
                    tag[0])
                Ball_list.append(('/'+ tag[1], Ball_location,'/'+ tag[0]))
                print "Found " + str(tag[1]) + " at " + str(tag[0])
            except(tf.Exception):
                print "Could not find " + str(tag[1]) + " at " + str(tag[0])
                continue
        #print Ball_list
        #print "Opening Right Camera"
        #CameraClose(left_hand_camera)
        #CameraOpen(right_hand_camera)
        #rospy.sleep(2.0)
        #print "Moving left arm out of the way"
        #MoveTo(left_arm, (0.6, 0.2, .2), (0.70710678118,0.70710678118,0.0,0.0))
        #print "Basket Location: "
        #print Basket_location
        print "Finding Basket with right arm"
        Basket_location = FindBasket(Basket_location, '/ar_marker_8', '/Basket')
        print Basket_location
        for t in Ball_list:
            print "Picking up " + str(t[0]) + " at " + str(t[2])
            PickUpBall(t[1], t[0])
            print "Dropping " + str(t[0]) + " at " + str(t[2])
            DropBallInBasket(Basket_location, '/Basket')
        print "Complete! Repeating in 7 seconds..."
        rospy.sleep(7.0)
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #continue
        rate.sleep()