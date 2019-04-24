#! /usr/bin/env python

import numpy as np
import scipy.io as sio
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from dbw_mkz_msgs.msg import SteeringCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringReport


class PID(object):
    def __init__(self, Kp=10, Kd=2, Ki=2, integral_max=500, output_max=1, output_min=0, period=0.04):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral_max = integral_max
        self.output_max= output_max
        self.output_min = output_min
        self.diff_before = None
        self.diff_integral = 0
        self.period = period
    
    def step(self, input_speed, standard_speed = 10):
        diff = standard_speed - input_speed
        self.diff_integral = np.clip(self.diff_integral + diff * self.period, -self.integral_max, self.integral_max)
        if self.diff_before is None:
            self.diff_before = diff
        output = self.Kp * diff + self.Ki * self.diff_integral + self.Kd * (diff - self.diff_before) / self.period
        self.diff_before = diff
        output = np.clip(output, self.output_min, self.output_max)
        return output    

def imu_callback(msg):
    global omega, controllerstate, steering_cmd, message_pub
    if omega is None:
        omega = msg.angular_velocity.z
    else:
        alpha = 1
        omega = msg.angular_velocity.z * alpha + (1 - alpha) * omega
    
    omega_desired = 0.1
    omega_error = omega_desired - omega
    # steering_cmd.steering_wheel_angle_cmd = pid_angle.step(omega, omega_desired)
    controller_input_vector = np.array([[omega_error]])
    controllerstate = controllerA.dot(controllerstate) + controllerB.dot(controller_input_vector)
    output = controllerC.dot(controllerstate) + controllerD.dot(controller_input_vector)
    steering_cmd.steering_wheel_angle_cmd = np.clip(output[0,0], -4, 4);
    steering_pub.publish(steering_cmd)



def reportCallback(msg):
    global pid_speed
    speed = msg.speed
    throttle_cmd.pedal_cmd = pid_speed.step(speed, standard_speed=10)
    throttle_pub.publish(throttle_cmd)

rospy.init_node("my_step_controller")

rospy.Subscriber("/vehicle/imu/data_raw", Imu, imu_callback)
rospy.Subscriber("/vehicle/steering_report", SteeringReport, reportCallback)

throttle_pub = rospy.Publisher("/vehicle/throttle_cmd", ThrottleCmd, queue_size=1)
steering_pub = rospy.Publisher("/vehicle/steering_cmd", SteeringCmd, queue_size=1)



steering_cmd = SteeringCmd()
steering_cmd.steering_wheel_angle_velocity = 0;
steering_cmd.clear = False;
steering_cmd.ignore = False;
steering_cmd.enable = True;
steering_cmd.count = 0;
steering_cmd.quiet = False;
omega = None

controller_dict = sio.loadmat('/home/owen/Desktop/play_ground/src/system_identification_car/scripts/matlab_omega_control/H_inf_control_matrix.mat')
controllerA = controller_dict['KA']
controllerB = controller_dict['KB']
controllerC = controller_dict['KC']
controllerD = controller_dict['KD']
print(controllerA.shape[0])

controllerstate = np.zeros([controllerA.shape[0],1]);
# pid_angle = PID(3.4, 0.93, 3.1, output_max=4, output_min=-4)



throttle_cmd = ThrottleCmd()
throttle_cmd.pedal_cmd_type = throttle_cmd.CMD_PERCENT;
throttle_cmd.pedal_cmd = 0;
throttle_cmd.enable = True;
throttle_cmd.ignore = False;
throttle_cmd.clear = False;
throttle_cmd.count = 0;

pid_speed = PID()


rospy.spin()