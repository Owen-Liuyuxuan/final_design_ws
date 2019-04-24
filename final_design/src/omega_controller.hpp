#include "numpy.hpp"

#include "ros/ros.h"
#include <ros/package.h>
#include "pid.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include "dbw_mkz_msgs/ThrottleCmd.h"
#include "dbw_mkz_msgs/SteeringCmd.h"
#include "dbw_mkz_msgs/BrakeCmd.h"
#include "dbw_mkz_msgs/SteeringReport.h"
#include <ros/package.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

//#include <Eigen/Core>
//#include <Eigen/Dense>
#include <iostream>
#pragma once

template <typename T>
void print_vector(const std::vector<T> vec)
{
    std::cout << "[";
    for (int i = 0; i < vec.size(); i++)
    {
        std::cout << vec[i];
        if (i != vec.size() - 1)
        {
            std::cout << ",";
        }
        else
        {
            std::cout << "]" << std::endl;
        }
    }
}

Eigen::MatrixXd vector2matrix(const std::vector<double> vec, int rows, int cols)
{
    Eigen::MatrixXd A(rows, cols);
    for(int i = 0; i < rows; i ++)
    {
        for(int j = 0; j < cols; j++)
        {
            A(i,j) = vec[cols * i + j];
        }
    }
    return A;
}

class OmegaControllerNode
{
    public:
    OmegaControllerNode(){;}
    OmegaControllerNode(ros::NodeHandle &n)
    {
        std::string folder_name = ros::package::getPath("rambot_controller");
        std::cout<<folder_name<<std::endl;
        //Publishing steering command and throttle command
        steering_pub = n.advertise<dbw_mkz_msgs::SteeringCmd>("/vehicle/steering_cmd", 1);
        throttle_pub = n.advertise<dbw_mkz_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 1);

        speed_sub = n.subscribe("/vehicle/steering_report", 1, &OmegaControllerNode::steering_report_callback, this);
        imu_sub = n.subscribe("/vehicle/imu/data_raw", 1, &OmegaControllerNode::imu_callback, this);
        twist_sub = n.subscribe("/vehicle/cmd_vel", 1, &OmegaControllerNode::twist_callback, this);

        std::string path = ros::package::getPath("system_identification_car");
        controllerpath = path + "/scripts/matlab_omega_control/";
        n.param<std::string>("/omega_controller/controller_type", controller_type,
        "proposed");
        n.param<double>("/omega_controller/steering_ratio", steering_ratio, 1);
        n.param<double>("/omega_controller/wheelbase_length", wheelbase_length, 1.65);
        n.param<double>("/omega_controller/max_steering_output", max_steering_output, 4);

        std::cout<<controller_type<<std::endl;

        loadController(controllerpath);
    }
    void run()
    {
        //wait for other nodes
        ros::Rate loop_rate(FREQUENCY);
        while(!imu_init || !twist_init)
        {
            loop_rate.sleep();
            ros::spinOnce();
        }
        while(ros::ok())
        {
            loop_rate.sleep();
            ros::spinOnce();
            float steering_output;
            
            if (speed < 0.1 && speed_cmd < 0.2)
                steering_output = 0;
            else
            {
                if (controller_type.compare("pid") == 0)
                    steering_output = pid_angle.step((angular_speed_cmd - angular_velocity)) + steering_ratio * openloop_steering();
                if (controller_type.compare("proposed") == 0)
                    steering_output = controller_output_once(angular_speed_cmd ,(angular_speed_cmd - angular_velocity));
                if (controller_type.compare("official") == 0)
                {
                    steering_output = steering_ratio * openloop_steering();
                }
            }   
            float speed_output = pid_instance_speed.step(speed_cmd - speed);

            dbw_mkz_msgs::ThrottleCmd throttle_cmd;
            throttle_cmd.pedal_cmd_type = throttle_cmd.CMD_PERCENT;
            throttle_cmd.pedal_cmd = 0;
            if(speed_output > 0)
                throttle_cmd.pedal_cmd = speed_output;
            throttle_cmd.enable = true;
            throttle_cmd.ignore = false;
            throttle_cmd.clear = false;
            throttle_cmd.count = 0;
            throttle_pub.publish(throttle_cmd);

            dbw_mkz_msgs::SteeringCmd steering_cmd;
            steering_cmd.steering_wheel_angle_velocity = 0;
            steering_cmd.clear = false;
            steering_cmd.ignore = false;
            steering_cmd.enable = true;
            steering_cmd.count = 0;
            steering_cmd.quiet = false;
            steering_cmd.steering_wheel_angle_cmd = steering_output;
            steering_pub.publish(steering_cmd);
        }
    }

    private:
    ros::Publisher steering_pub;
    ros::Publisher throttle_pub;
    ros::Subscriber speed_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber twist_sub;
    const double FREQUENCY = 25;
    bool imu_init = false;
    bool twist_init = false;

    PID pid_instance_speed = PID(1/FREQUENCY, 10, 2, 2, 500);
    PID pid_angle = PID(1/FREQUENCY, 1, 11.6, 0.02, 3000, max_steering_output, -max_steering_output);
    float angular_velocity = 0;
    float speed = 0;
    float speed_cmd = 10;
    float angular_speed_cmd = 0.1;

    std::string controllerpath, controller_type;
    std::vector<double> KA_vec, KB_vec, KC_vec, KD_vec;
    Eigen::MatrixXd KA, KB, KC, KD;
    Eigen::MatrixXd state;
    int n_state, n_input, n_output;

    double steering_ratio, wheelbase_length;
    double max_steering_output;

    void steering_report_callback(const dbw_mkz_msgs::SteeringReportConstPtr &msg)
    {
        speed = msg->speed;
    }
    void imu_callback(const sensor_msgs::ImuConstPtr &msg)
    {
        imu_init=true;
        angular_velocity = msg->angular_velocity.z;
    }
    void twist_callback(const geometry_msgs::TwistConstPtr &msg)
    {
        twist_init = true;
        speed_cmd = float(msg->linear.x);
        angular_speed_cmd = float(msg->angular.z);
    }
    double controller_output_once(const double omega_command, const double omega_error)
    {
        Eigen::MatrixXd input;
        if(n_input == 2)
        {
            input = Eigen::MatrixXd(2,1);
            input(0,0) = omega_command;
            input(1,0) = omega_error;
        }
        if(n_input == 1)
        {
            input = Eigen::MatrixXd(1,1);
            input(0,0) = omega_error;
        }

        state = KA * state + KB * input;
        Eigen::MatrixXd output = (KC * state + KD * input);
        double raw_output = steering_ratio * (output(0,0) + openloop_steering());

        return pid_utils::clip(raw_output, max_steering_output, -max_steering_output);

    }
    double openloop_steering()
    {
        if(speed > 0.1)
        {
            return atan(wheelbase_length * angular_speed_cmd / speed);
        }
        if(speed_cmd > 0.2)
        {
            return atan(wheelbase_length * angular_speed_cmd / speed_cmd);
        }
        return 0;
    }
    void loadController(std::string controllerpath)
    {
        aoba::LoadArrayFromNumpy(controllerpath+"KA.npy", n_state, n_state, KA_vec);
        KA = vector2matrix(KA_vec, n_state, n_state);
        
        aoba::LoadArrayFromNumpy(controllerpath+"KB.npy", n_state, n_input, KB_vec);
        KB = vector2matrix(KB_vec,n_state, n_input);

        aoba::LoadArrayFromNumpy(controllerpath+"KC.npy", n_output, n_state, KC_vec);
        KC = vector2matrix(KC_vec,n_output, n_state);
        
        aoba::LoadArrayFromNumpy(controllerpath+"KD.npy", n_output, n_input, KD_vec);
        KD = vector2matrix(KD_vec,n_output,n_input);

        state = Eigen::MatrixXd::Zero(n_state, 1);
    }
};