#! /usr/bin/env python

import cv2
import numpy as np
import pandas as pd

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from tf2_msgs.msg import TFMessage
import tf

from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import ThrottleCmd
from dbw_mkz_msgs.msg import SteeringCmd
from sensor_msgs.msg import Imu


num_data = 500
num_dim = 9
buffer_index = 0
waiting_index = 0

data_buffer = None
throttle_cmd = None
steering_cmd = None
omega = None
command_omega = None
command_v = None

def image_callback(data):
    """
    data: Image
    """    
    global num_data, buffer_index, throttle_cmd, steering_cmd, omega, command_omega, command_v, waiting_index

    if(buffer_index >= num_data):
        print("finished")
        if buffer_index == num_data:
            data_buffer.to_csv("/home/owen/Desktop/play_ground/src/car_information_listener/scripts/pictures/data.csv")
            buffer_index += 1 
        return 
    
    if waiting_index < 10:
        waiting_index += 1
        return
    else:
        waiting_index = 0
        new_image = my_bridge.imgmsg_to_cv2(data, "bgr8")
        (x, y, theta) = get_TF()
        if x is None:
            return 

        filename = "/home/owen/Desktop/play_ground/src/car_information_listener/scripts/pictures/test_image" + str(buffer_index) + ".png"
        cv2.imwrite(filename, new_image)

        data_buffer.iloc[buffer_index, 0] = filename
        data_buffer.iloc[buffer_index, 1:] = [x, y, theta, command_v, command_omega, throttle_cmd, steering_cmd, omega]

        buffer_index += 1


    
    
    

def get_TF():
    global listener
    try:
        (trans, rot) = listener.lookupTransform(
            '/world', 'vehicle/base_footprint', rospy.Time(0))
        x = trans[0]
        y = trans[1]
        theta = 2 * atan2(rot[2], rot[3])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return (None, None, None)
    return (x, y, theta)

def twist_callback(msg):
    global command_v, command_omega
    command_v = msg.linear.x;
    command_omega = msg.angular.z;

def throttle_callback(msg):
    global throttle_cmd
    throttle_cmd = msg.pedal_cmd

def steer_callback(msg):
    global steering_cmd
    steering_cmd = msg.steering_wheel_angle_cmd

def imu_callback(msg):
    global omega
    omega = msg.angular_velocity.z

def init_data_buffer():
    A = pd.Series(["wtf" + str(i) for i in range(5000)],dtype=str)
    A = pd.DataFrame(A)
    B = np.zeros([num_data, num_dim - 1])
    B = pd.DataFrame(B)
    C = pd.concat([A,B], axis=1)
    C.columns  = [str(i) for i in range(num_dim)]
    return C

my_bridge = CvBridge()
data_buffer = init_data_buffer()
rospy.init_node("image_car_listener")
rospy.Subscriber("/vehicle/cmd_vel", Twist, twist_callback)
rospy.Subscriber("/vehicle/throttle_cmd", ThrottleCmd, throttle_callback,queue_size=1)
rospy.Subscriber("/vehicle/steering_cmd", SteeringCmd, steer_callback, queue_size=1)
rospy.Subscriber("/vehicle/imu/data_raw", Imu, imu_callback, queue_size=1)
rospy.Subscriber("/vehicle/front_camera/image_raw", Image, image_callback)
listener = tf.TransformListener()

def myhook():
      if buffer_index < num_data:
          shorter_buffer = data_buffer[:buffer_index]
          shorter_buffer.to_csv("/home/owen/Desktop/play_ground/src/car_information_listener/scripts/pictures/data.csv")

rospy.on_shutdown(myhook)


rospy.spin()
