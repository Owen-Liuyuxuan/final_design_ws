#pragma once
// ROS Relevant Libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

// Eigen Libraries, opencv, std Libraries
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

// Customed classes
#include "imageProcessor.hpp"
#include "roadTracker.hpp"
#include "transform_pic_veh.h"

class laneDetectorNode
{
public:
  laneDetectorNode(ros::NodeHandle &n)
  {
    _pub = n.advertise<nav_msgs::Path>("/vehicle/target_path", 1);
    _pub2 = n.advertise<nav_msgs::Path>("yellow_lane", 1);
    _pub3 = n.advertise<nav_msgs::Path>("/white_lane", 1);
    _sub = n.subscribe("/vehicle/front_camera/image_raw", 1, &laneDetectorNode::imageCallback, this);
    init_transformer();
  }
  laneDetectorNode()
  {
    init_transformer();
  }
  void init_transformer()
  {
    // Transformer Init ->Here We can use transformer.transformToVehicle(const std::vector<cv::Point2d> image_path,
    // std::vector<cv::Point2d> &vehicle_path)
    Eigen::Matrix3d K;
    K << 476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0;
    // cout<<K<<endl;
    double quarternion1[4] = { 0, 0.295520206661, 0, 0.955336489126 };
    double quarternion2[4] = { -0.5, 0.5, -0.5, 0.5 };
    Eigen::Matrix3d R1, R2;
    _quarternion_2_matrix(quarternion1, R1);
    _quarternion_2_matrix(quarternion2, R2);

    Eigen::Vector3d T1;
    T1 << 2, 0, 1.3;
    // Notice: Data from the tf tree shows how p = [R T]p', p' is the coordinate in child frame.
    // We have to do some kinds of inverse here.
    _transformer = imageVehicleTransformer(R2.transpose() * R1.transpose(), -R2.transpose() * R1.transpose() * T1, K);
    _transformer.initTransforms();
  }
  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    // msg -> cv::Mat

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      return;
    }

    cv::Mat image = cv_ptr->image;

    // get  yellow_mask and white_mask
    _image_processor.process_image(image);

    // transform to point lists;
    std::vector<cv::Point> white_point_list_image, yellow_point_list_image;
    cv::findNonZero(_image_processor._white_mask, white_point_list_image);
    cv::findNonZero(_image_processor._yellow_mask, yellow_point_list_image);
    std::vector<cv::Point2d> white_point_list, yellow_point_list;
    _transformer.transformToVehicle(white_point_list_image, white_point_list, 0, 80);
    _transformer.transformToVehicle(yellow_point_list_image, yellow_point_list, 0, 80);

    // lane tracker update
    _lane_tracker.update_white_lane(white_point_list);
    _lane_tracker.update_yellow_lane(yellow_point_list);

    // publish the target path
    publish_target_path();
  }
  void publish_target_path()
  {
    if (_lane_tracker._yellow_lane_init && _lane_tracker._white_lane_init)
    {
      nav_msgs::Path msg, msg_yellow, msg_white;
      const int PATH_LENGTH = 40;
      msg.poses.resize(PATH_LENGTH);
      msg_yellow.poses.resize(PATH_LENGTH);
      msg_white.poses.resize(PATH_LENGTH); 
      _lane_tracker.get_target_road();
      for (int i = 0; i < PATH_LENGTH; i++)
      {
        double x = i / (PATH_LENGTH / 18.0) + 4;
        double y_target = np::polyeval(_lane_tracker.target_lane, x);
        double y_yellow = np::polyeval(_lane_tracker._yellow_lane._coefficients, x);
        double y_white = np::polyeval(_lane_tracker._white_lane._coefficients, x);
        // double y_yellow = np::polyeval(_lane_tracker._yellow_lane._coefficients, x);
        // double y_white = np::polyeval(_lane_tracker._white_lane._coefficients, x);
        // double y_target = (_lane_tracker._yellow_weight * y_yellow + _lane_tracker._white_weight * y_white) /
        //                   (_lane_tracker._yellow_weight + _lane_tracker._white_weight);
        msg.poses[i].pose.position.x = x;
        msg_yellow.poses[i].pose.position.x = x;
        msg_white.poses[i].pose.position.x = x;
        msg.poses[i].pose.position.y = y_target;
        msg_yellow.poses[i].pose.position.y = y_yellow;
        msg_white.poses[i].pose.position.y = y_white;
        msg.poses[i].pose.position.z = -0.33;
        msg_yellow.poses[i].pose.position.z = -0.33;
        msg_white.poses[i].pose.position.z = -0.33;
      }
      // msg.header.frame_id = "vehicle/base_footprint";
      msg.header.frame_id = "vehicle/base_link";
      msg_yellow.header.frame_id = "vehicle/base_link";
      msg_white.header.frame_id = "vehicle/base_link";
      _pub.publish(msg);
      _pub2.publish(msg_yellow);
      _pub3.publish(msg_white);
    }
  }
  imageProcessor _image_processor;
  imageVehicleTransformer _transformer;
  roadTracker _lane_tracker;
  ros::Publisher _pub;
  ros::Publisher _pub2;
  ros::Publisher _pub3;
  ros::Subscriber _sub;

private:
  void _quarternion_2_matrix(const double *quarternion, Eigen::Matrix3d &R)
  {
    // quarternion = [x, y, z, w]  -> qi, qj, qk, qr
    double qi = quarternion[0], qj = quarternion[1], qk = quarternion[2], qr = quarternion[3];
    R << 1 - 2 * (qj * qj + qk * qk), 2 * (qi * qj - qk * qr), 2 * (qi * qk + qj * qr), 2 * (qi * qj + qk * qr),
        1 - 2 * (qi * qi + qk * qk), 2 * (qj * qk - qi * qr), 2 * (qi * qk - qj * qr), 2 * (qj * qk + qi * qr),
        1 - 2 * (qi * qi + qj * qj);
  }
};
