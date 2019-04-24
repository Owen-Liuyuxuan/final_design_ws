#! /usr/bin/env python

#include "geometry_msgs/Twist.h"

import rospy
from geometry_msgs.msg import Twist
import numpy as np

rospy.init_node("random_commander", anonymous = True)
pub = rospy.Publisher("/vehicle/cmd_vel", Twist, queue_size =1)
rate  = rospy.Rate(30)
v = 0
omega = 0
index = 0

while not rospy.is_shutdown():
    msg = Twist()
    if index % 10 == 0:
        v = 10 + 1 * np.random.randn()
        omega = 0.1 + 0.4 * np.random.randn()
    msg.linear.x = v
    msg.angular.z = omega
    if index  < 9915017392:
        index += 1
    else:
        index = 0
    pub.publish(msg)
    rate.sleep()