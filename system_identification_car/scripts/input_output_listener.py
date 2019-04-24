#! /usr/bin/env python

#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/SteeringCmd.h"

import numpy as np
from tf2_msgs.msg import TFMessage
import rospy
from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from sensor_msgs.msg import Imu
import rospkg

import tf
from math import *

rospy.init_node("tf_listener")
rospack = rospkg.RosPack()


listener = tf.TransformListener()
num_data = 300 * 25
dim_data = 8
temp_buffer = np.zeros([num_data, dim_data]) #x, y, theta, v_command, omega_command, throttle_cmd, steering_cmd, omega
buffer_index = 0

throttle_cmd = None
steering_cmd = None
omega = None

def twist_callback(msg):
    global buffer_index, temp_buffer, num_data, throttle_cmd, steering_cmd, omega
    if omega is None or steering_cmd is None or throttle_cmd is None:
        return None
    
    if buffer_index < num_data:
        try:
            (trans, rot) = listener.lookupTransform(
                '/world', 'vehicle/base_footprint', rospy.Time(0))
            temp_buffer[buffer_index, 0] = trans[0]
            temp_buffer[buffer_index, 1] = trans[1]
            temp_buffer[buffer_index, 2] = 2 * atan2(rot[2], rot[3])
            temp_buffer[buffer_index, 3] = msg.linear.x;
            temp_buffer[buffer_index, 4] = msg.angular.z;
            temp_buffer[buffer_index, 5] = throttle_cmd;
            temp_buffer[buffer_index, 6] = steering_cmd;
            temp_buffer[buffer_index, 7] = omega;
            buffer_index += 1
            if buffer_index%30 == 0:
                print(buffer_index)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            print("miss")
    else:
        if(buffer_index == num_data):
            pack_dir = rospack.get_path("system_identification_car")
            np.save(pack_dir + "/scripts/python_data/running_data.npy", temp_buffer)
            buffer_index += 1
            print("stored")
        print("buffer full")

def throttle_callback(msg):
    global throttle_cmd
    throttle_cmd = msg.pedal_cmd

def steer_callback(msg):
    global steering_cmd
    steering_cmd = msg.steering_wheel_angle_cmd

def imu_callback(msg):
    global omega
    omega = msg.angular_velocity.z

rospy.Subscriber("/vehicle/cmd_vel", Twist, twist_callback)
rospy.Subscriber("/vehicle/throttle_cmd", ThrottleCmd, throttle_callback,queue_size=1)
rospy.Subscriber("/vehicle/steering_cmd", SteeringCmd, steer_callback, queue_size=1)
rospy.Subscriber("/vehicle/imu/data_raw", Imu, imu_callback, queue_size=1)

def myhook():
      if buffer_index < num_data:
          shorter_buffer = temp_buffer[:buffer_index, :]
          pack_dir = rospack.get_path("system_identification_car")
          np.save(pack_dir + "/scripts/python_data/running_data.npy", shorter_buffer)

rospy.on_shutdown(myhook)


rospy.spin()

