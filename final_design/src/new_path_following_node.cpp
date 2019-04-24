#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "dbw_mkz_msgs/SteeringReport.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

#include <vector>
#include <iostream>
#include <cmath>
typedef unsigned int uint;
using namespace std;
double steering_wheel_angle, steering_wheel_angle_cmd, speed;
bool is_steering_report_init = false;
bool is_path_init = false;

const int RLS_ORDER = 4;
Eigen::MatrixXd omega_responsive_coeffs(RLS_ORDER, 1);
Eigen::MatrixXd RLS_phi;//y-1, y-2, u-1, u-2
Eigen::MatrixXd RLS_P(RLS_ORDER, RLS_ORDER);
double lambda = 0.9999;

MPC mpc;
double output_omega=0;
Eigen::VectorXd coeffs;

double current_angular_velocity=0;

inline void push_command(double omega_cmd)
{
    RLS_phi(3, 0) = RLS_phi(2, 0);
    RLS_phi(2, 0) = omega_cmd;
}
inline void push_observation(double omega_observe)
{
    RLS_phi(1, 0) = RLS_phi(0, 0);
    RLS_phi(0, 0) = omega_observe;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}
void steering_report_cb(const dbw_mkz_msgs::SteeringReport::ConstPtr &msg)
{
    steering_wheel_angle = msg->steering_wheel_angle;
    steering_wheel_angle_cmd = msg->steering_wheel_angle_cmd;
    speed = msg->speed;
    is_steering_report_init = true;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    current_angular_velocity = msg->angular_velocity.z;
    push_observation(current_angular_velocity);
}

void path_cb(const nav_msgs::Path::ConstPtr &msg)
{
    if(! is_steering_report_init)
        return;

    if(msg->poses.size() < 4)
        return;

    is_path_init = true;
    Eigen::VectorXd xvals(msg->poses.size());
    Eigen::VectorXd yvals(msg->poses.size());
    for (uint i = 0; i < msg->poses.size(); i++)
    {
        xvals[i] = msg->poses[i].pose.position.x;
        yvals[i] = msg->poses[i].pose.position.y;
    }
    coeffs = polyfit(xvals, yvals, 3);
}

//There is calculation error in RLS_estimation. We should take the last two measurements and the closest input command to get current 
//estimation, instead of the last three inputs. Check again!
void RLS_init()
{
    omega_responsive_coeffs = Eigen::MatrixXd::Zero(RLS_ORDER, 1);
    omega_responsive_coeffs<<1.734, -0.783, 0.04775, 0;
    RLS_phi = Eigen::MatrixXd::Zero(RLS_ORDER, 1);
    RLS_P = Eigen::MatrixXd::Identity(RLS_ORDER, RLS_ORDER) * 0.001;
}
void RLS_estimation()
{
    Eigen::MatrixXd K = RLS_P * RLS_phi/(lambda + (RLS_phi.transpose() * RLS_P * RLS_phi)(0,0));
    double error = current_angular_velocity - (omega_responsive_coeffs.transpose() * RLS_phi)(0,0);
    omega_responsive_coeffs += K * error;
    RLS_P = (Eigen::MatrixXd::Identity(RLS_ORDER, RLS_ORDER) - K*RLS_phi.transpose()) * RLS_P/(lambda);
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "path_following");
    ros::NodeHandle n;
    ros::Publisher pub_handle1 = n.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    ros::Subscriber sub_handle1 = n.subscribe("/vehicle/steering_report", 1, steering_report_cb);
    ros::Subscriber sub_handle2 = n.subscribe("/vehicle/target_path", 1, path_cb);
    ros::Subscriber sub_handle3 = n.subscribe("/vehicle/imu/data_raw", 1, imu_callback);
    ros::Rate loop_rate(25);

    RLS_init();
    
    cout<<RLS_P<<endl;
    
    while(!(is_path_init && is_steering_report_init))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    int index = 0;
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        RLS_estimation();
        if(index % 30 == 0)
            cout<<omega_responsive_coeffs<<endl;
        index ++;

        const Eigen::VectorXd state = Eigen::Vector3d::Zero();
        vector<double> result = mpc.Solve(state, coeffs, RLS_phi, omega_responsive_coeffs, speed);
        output_omega = result[0];

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = V;
        cmd_vel.angular.z = output_omega;
        pub_handle1.publish(cmd_vel);
        push_command(output_omega);
    }
}