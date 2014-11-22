#!/usr/bin/env python
import rospy
from baxter_interface import gripper as baxter_gripper

rospy.init_node('gripper_test')

#Set up the left gripper
left_gripper = baxter_gripper.Gripper('left')

def offset_holding(gripper, offset):
    current = gripper.parameters()['holding_force']
    gripper.set_holding_force(current + offset)

#Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
left_gripper.calibrate()
rospy.sleep(2.0)

#Close the left gripper
print('Closing...')
left_gripper.close(block=True)
rospy.sleep(0.5)
offset_holding(left_gripper, 100.0)

#Open the left gripper
# print('Opening...')
# left_gripper.open(block=True)
# rospy.sleep(1.0)
# print('Done!')
