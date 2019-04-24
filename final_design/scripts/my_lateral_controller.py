#! /usr/bin/env python

import rospy
import numpy as np
import scipy.io as sio
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

import tf
from math import *
import time

rospy.init_node("my_lateral_controller")

controller_dict = sio.loadmat('/home/owen/Desktop/play_ground/src/system_identification_car/scripts/matlab_data/robust_discrete_controller.mat')
controllerA = controller_dict['KA']
controllerB = controller_dict['KB']
controllerC = controller_dict['KC']
controllerD = controller_dict['KD']

controllerstate = np.zeros([controllerA.shape[0],1]);

steering_cmd = SteeringCmd()
steering_cmd.steering_wheel_angle_velocity = 0;
steering_cmd.clear = False;
steering_cmd.ignore = False;
steering_cmd.enable = True;
steering_cmd.count = 0;
steering_cmd.quiet = False;
omega = None
init = False

def imu_callback(msg):
    global omega
    omega = msg.angular_velocity.z

def path_callback(msg):
    global omega, controllerstate, steering_cmd, message_pub
    SPEED = 15
    if omega is None:
        return
    poses = msg.poses
    x_list = []
    y_list = []
    for pose in poses:
        x_list.append(pose.pose.position.x)
        y_list.append(pose.pose.position.y)
    x_array = np.array(x_list)
    y_array = np.array(y_list)
    coefficients = np.polyfit(x_array, y_array, 3)
    error_y = -coefficients[-1]
    error_phi = -atan2(coefficients[-2], 1)
    desired_phi_dot = - 1.0/(1 + coefficients[-2] ** 2) * 2 * coefficients[-3] * SPEED

    controller_input_vector = np.array([[error_y], [error_phi], [omega],[desired_phi_dot]])
    controllerstate = controllerA.dot(controllerstate) + controllerB.dot(controller_input_vector)
    output = controllerC.dot(controllerstate) + controllerD.dot(controller_input_vector)
    
    steering_cmd.steering_wheel_angle_cmd = -np.clip(output[0,0], -6, 6);
    message_pub.publish(steering_cmd)
    


rospy.Subscriber("/vehicle/imu/data_raw", Imu, imu_callback, queue_size=1)

rospy.Subscriber("/vehicle/target_path", Path, path_callback, queue_size=1)

message_pub = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)


rospy.spin()
