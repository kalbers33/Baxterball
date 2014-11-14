#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
if __name__ == '__main__':
    rospy.init_node('tf_baxterball')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    print "start"
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/ar_marker_1', '/left_gripper', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print trans
        print rot
        rate.sleep()

