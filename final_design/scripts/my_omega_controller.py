#! /usr/bin/env python

import numpy as np
import scipy.io as sio
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from dbw_mkz_msgs.msg import SteeringCmd

def twist_callback(msg):
    global omega, controllerstate, steering_cmd, message_pub
    if omega is None:
        return
    omega_desired = msg.angular.z
    omega_error = omega_desired - omega
    #controller_input_vector = np.array([[omega_error],[omega_desired]])
    controller_input_vector = np.array([[omega_error]])
    controllerstate = controllerA.dot(controllerstate) + controllerB.dot(controller_input_vector)
    output = controllerC.dot(controllerstate) + controllerD.dot(controller_input_vector)
    print(controllerstate)
    steering_cmd.steering_wheel_angle_cmd = np.clip(output[0,0], -4, 4);
    message_pub.publish(steering_cmd)

def imu_callback(msg):
    global omega
    if omega is None:
        omega = msg.angular_velocity.z
    else:
        alpha = 1
        omega = msg.angular_velocity.z * alpha + (1 - alpha) * omega 

rospy.init_node("my_omega_controller")

rospy.Subscriber("/vehicle/cmd_vel", Twist, twist_callback)
rospy.Subscriber("/vehicle/imu/data_raw", Imu, imu_callback)

message_pub = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)

controller_dict = sio.loadmat('/home/owen/Desktop/play_ground/src/system_identification_car/scripts/matlab_omega_control/H_2_control_matrix.mat')
controllerA = controller_dict['KA']
controllerB = controller_dict['KB']
controllerC = controller_dict['KC']
controllerD = controller_dict['KD']
print(controllerA.shape[0])

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


rospy.spin()