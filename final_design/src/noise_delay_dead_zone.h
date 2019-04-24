#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/SteeringReport.h"

#include <random>
float pass_deadzone(float input_number, float dead_zone_start, float dead_zone_end)
{
    if(input_number < dead_zone_start)
        return input_number + dead_zone_start;
    if(input_number > dead_zone_end)
        return input_number - dead_zone_end;
    else
        return 0;
}
class NoiseNode
{
    public:
    NoiseNode(){;}
    NoiseNode(ros::NodeHandle &n)
    {
        std::random_device rd;
        gen = std::mt19937(rd());

        steering_pub = n.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
        imu_pub = n.advertise<sensor_msgs::Imu>("/processed/imu/data_raw", 1);

        imu_sub = n.subscribe("/vehicle/imu/data_raw", 1, &NoiseNode::imu_callback, this);
        steering_sub = n.subscribe("/unprocessed/steering_cmd", 1, &NoiseNode::steering_callback, this);

        n.param<float>("/noise_adder/noise_imu", noise_imu, 0.05);
        n.param<float>("/noise_adder/alpha", alpha, 0.8);
        n.param<float>("/noise_adder/noise_steering", noise_steering, 0.03);
        n.param<float>("/noise_adder/deadzone_length", deadzone_length, 0.05);
        
    }
    void run(){ros::spin();}

    private:

    ros::Publisher steering_pub;
    ros::Publisher imu_pub;

    ros::Subscriber imu_sub;
    ros::Subscriber steering_sub;
    
    bool is_imu_init = false;
    float noise_imu = 0.001;
    float last_omega = 0;
    float alpha = 0.3297;//A = tf(1, [0.1, 1]), c2d(A, 0.04);

    bool is_steering_init = false;
    float deadzone_length = 0.03;
    float noise_steering = 0.01;
    float last_steering = 0;
    // const float alpha_steering = 0.1813;//A = tf(1, [0.2, 1]), c2d(A, 0.04);

    std::mt19937 gen;
    void imu_callback(const sensor_msgs::ImuConstPtr &msg)
    {
        static std::uniform_real_distribution<float> measurement_noise(-noise_imu, noise_imu);
        float noise = measurement_noise(gen);
        sensor_msgs::Imu processed_message;
        processed_message.header = msg->header;
        float measured_omega = msg->angular_velocity.z + noise;
        if(!is_imu_init)
        {
            is_imu_init = true;
            last_omega = measured_omega;
        }
        processed_message.angular_velocity.z = alpha * measured_omega + (1 - alpha) * last_omega;
        imu_pub.publish(processed_message);
    }
    void steering_callback(const dbw_mkz_msgs::SteeringCmdConstPtr &msg)
    {
        static std::normal_distribution<float> actuation_noise(0, noise_steering);//float sample = d(gen);;
        float noise = actuation_noise(gen);
        dbw_mkz_msgs::SteeringCmd processed_message;
        processed_message.enable = msg->enable;
        processed_message.ignore = msg->ignore;
        processed_message.quiet = msg->quiet;
        processed_message.clear = msg->clear;
        float actual_steeringCmd = pass_deadzone(msg->steering_wheel_angle_cmd, -deadzone_length/2, deadzone_length/2);
        processed_message.steering_wheel_angle_cmd = actual_steeringCmd;
        steering_pub.publish(processed_message);
    }
};

