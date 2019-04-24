#! /usr/bin/env python

import cv2
import numpy as np
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
import rospy
import tf

number_of_received = 0
number_to_receive = 10
index = 0








rospy.init_node("tf_listener")
listener = tf.TransformListener()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
	try:
		(trans, rot) = listener.lookupTransform(
		    '/world', 'vehicle/base_footprint', rospy.Time(0))
		print("x:")
		print(trans[0])
		print("y:")
		print(trans[1])
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		pass
		print("didn't get anything")
	rate.sleep()
