#! /usr/bin/env python

import rospy
import numpy as np
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringReport
from nav_msgs.msg import Path

rospy.init_node("my_lon_controller")

throttle_cmd = ThrottleCmd()
throttle_cmd.pedal_cmd_type = throttle_cmd.CMD_PERCENT;
throttle_cmd.pedal_cmd = 0;
throttle_cmd.enable = True;
throttle_cmd.ignore = False;
throttle_cmd.clear = False;
throttle_cmd.count = 0;
path_init = False

class PID(object):
    def __init__(self, Kp=10, Kd=2, Ki=2, integral_max=500, output_max=1, output_min=0, period=0.02):
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
        self.diff_integral = np.clip(self.diff_integral + diff * self.period, self.integral_max, -self.integral_max)
        if self.diff_before is None:
            self.diff_before = diff
        output = self.Kp * diff + self.Ki * self.diff_integral + self.Kd * (diff - self.diff_before) / self.period
        self.diff_before = diff
        return output

def path_callback(msg):
    global path_init
    path_init = True

def reportCallback(msg):
    global path_init
    if not path_init:
        return 
    global pid_speed
    speed = msg.speed
    throttle_cmd.pedal_cmd = pid_speed.step(speed)
    message_pub.publish(throttle_cmd)

pid_speed = PID()
message_pub = rospy.Publisher("/vehicle/throttle_cmd", ThrottleCmd, queue_size=1)
rospy.Subscriber("/vehicle/steering_report", SteeringReport, reportCallback, queue_size=1)
rospy.Subscriber("/vehicle/target_path", Path, path_callback, queue_size=1)

rospy.spin()
