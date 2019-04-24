#! /usr/bin/env python

import rospy
import numpy as np
import scipy.io as sio
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from sensor_msgs.msg import Imu

import tf
from math import *

rospy.init_node("tf_listener")

controller_dict = sio.loadmat('/home/owen/Desktop/play_ground/src/system_identification_car/scripts/matlab_data/discrete_controller_2.mat')
controllerA = controller_dict['KA']
controllerB = controller_dict['KB']
controllerC = controller_dict['KC']
controllerD = controller_dict['KD']

controllerstate = np.zeros([controllerA.shape[0],1]);

model_dict = sio.loadmat('/home/owen/Desktop/play_ground/src/system_identification_car/scripts/matlab_data/discrete_model_matrix.mat')
modelA = model_dict['robust_system_A']
modelB = model_dict['robust_system_B']
modelC = model_dict['robust_system_C']
modelD = model_dict['robust_system_D']
model_state = np.zeros([4, 1])

listener = tf.TransformListener()
throttle_cmd = None
steering_cmd = None
omega = None
init = False

def twist_callback(msg):
    global model_state, controllerstate, init
    if omega is None or steering_cmd is None or throttle_cmd is None:
        return None

    try:
        (trans, rot) = listener.lookupTransform(
           '/world', 'vehicle/base_footprint', rospy.Time(0))
        x = trans[0]
        y = trans[1]
        theta = 2 * atan2(rot[2], rot[3])
        v_cmd = msg.linear.x;
        omega_cmd = msg.angular.z;
        AMPLITUDE = 2.0
        WAVELENGTH = 200.0 + 50 * sin(2 * pi * x / (2500.0))
        w = 2 * pi / WAVELENGTH
        desired_y = AMPLITUDE * sin( w * x );
        desired_phi = atan2(w* AMPLITUDE * cos(w * x), 1)
        desired_phi_dot = - 1 / (1 + (w* AMPLITUDE * cos(w * x)) ** 2) * w * w * AMPLITUDE * sin(w * x) * v_cmd;
        error_y  =  y - desired_y;
        error_phi = theta - desired_phi;
        controller_input_vector = np.array([[error_y], [error_phi], [omega],[desired_phi_dot]])
        controllerstate = controllerA.dot(controllerstate) + controllerB.dot(controller_input_vector)
        output = controllerC.dot(controllerstate) + controllerD.dot(controller_input_vector)
        controller_output = output[0,0]

        print(controller_output, '  ', steering_cmd)
        #if init is False:
        #    init = np.array([[error_y], [error_phi],[error_y], [error_phi], [omega],[desired_phi_dot]])
        #model_input_vector = np.array([[desired_phi_dot], [steering_cmd]])
        #model_state = modelA.dot(model_state) + modelB.dot(model_input_vector)
        #output = modelC.dot(model_state) + modelD.dot(model_input_vector) - init
        #print("expected_output:", output[2:])
        #print("real_output:", np.array([[error_y], [error_phi], [omega],[desired_phi_dot]]))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("miss")

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

rospy.spin()
