#pragma once
#include <opencv2/imgproc/imgproc_c.h>
#include <algorithm>
#include <iostream>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <random>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"

#include "my_numpy.hpp"

const int ORDER = 3;
class RANSACMachine
{
public:
  RANSACMachine(const int order_polynomial = 2) : _order_polynomial(order_polynomial)
  {
    ;
  }
  double sample_once(const std::vector<cv::Point2d> non_zero_point_list, Eigen::VectorXd &src_coefficient_vector,
                     int number_points = ORDER + 1, double bearing = 0.3)
  {
    Eigen::VectorXd x_list(number_points);
    Eigen::VectorXd y_list(number_points);

    if (np::get_different_x_y_list(non_zero_point_list, x_list, y_list, number_points))
    {
      src_coefficient_vector = np::polyfit(x_list, y_list, _order_polynomial);

      return _get_agree_rate(src_coefficient_vector, non_zero_point_list, bearing);
    }
    else
    {
      return 0;
    }
  }
  double _get_agree_rate(const Eigen::VectorXd coeffs, const std::vector<cv::Point2d> non_zero_point_list,
                         double bearing)
  {
    for (int i = 0; i <= ORDER; i++)
    {
      if (i == 0)
      {
        if (fabs(coeffs[0]) > 6)
        {
          return 0;
        }
      }
      if (i == 1)
      {
        if (fabs(coeffs[1]) > 3)
        {
          return 0;
        }
      }
      if (i > 1)
      {
        if (fabs(coeffs[i]) > 0.05)
        {
          return 0;
        }
      }
    }
    double total_agreement = 0;

    for (int i = 0; i < non_zero_point_list.size(); i++)
    {
      double x = non_zero_point_list[i].x;
      double true_y = non_zero_point_list[i].y;
      double predicted_y = np::polyeval(coeffs, x);
      if (abs(true_y - predicted_y) < bearing)
        total_agreement += 1;
    }

    return total_agreement / non_zero_point_list.size();
  }
  int _order_polynomial;
};

class lane
{
public:
  lane(const int order_polynomial = 3, double filter_lambda = 0.7)
    : _order_polynomial(order_polynomial), _filter_lambda(filter_lambda)
  {
    _coefficients = Eigen::VectorXd::Zero(order_polynomial + 1);
    is_init = false;
  }
  void reinit()
  {
    is_init = false;
  }
  void update(const Eigen::VectorXd coeffs, const double weight)
  {
    if (is_init && weight == 0.5)
    {
      _coefficients = _coefficients * _filter_lambda + coeffs * (1 - _filter_lambda);
      //_coefficients = coeffs;
    }
    else
    {
      is_init = true;
      _coefficients = coeffs;
    }
  }

  Eigen::VectorXd _coefficients;
  bool is_init;
  int _order_polynomial;
  double _filter_lambda;
};

class roadTracker
{
public:
  roadTracker()
  {
    polyfitter = RANSACMachine(ORDER);
    _white_lane = lane(ORDER);
    _yellow_lane = lane(ORDER);
    _yellow_lane._filter_lambda = 0.4;
    _white_lane._filter_lambda = 0.5;
    _yellow_weight = 0.5;
    _white_weight = 0.5;
    _yellow_lane_init = false;
    _white_lane_init = false;
  }
  void get_target_road()
  {
    Eigen::VectorXd modified_white = _white_lane._coefficients;
    Eigen::VectorXd modified_yellow = _yellow_lane._coefficients;
    modified_white[0] -= 1.8;
    modified_yellow[0] += 1.8;
    target_lane =
        (_yellow_weight * modified_yellow + _white_weight * modified_white) / (_yellow_weight + _white_weight);
  }
  void update_white_lane(const std::vector<cv::Point2d> white_point_list, int number_points = ORDER + 15,
                         int max_iter = 40)
  {
    if (white_point_list.size() < ORDER + 1)
    {
      _white_weight *= 0.8;
      return;
    }

    if (white_point_list.size() < number_points)
      number_points = white_point_list.size();

    Eigen::VectorXd best_coeffs = _white_lane._coefficients;
    double best_agree_rate = 0;
    for (int i = 0; i < max_iter; i++)
    {
      Eigen::VectorXd temp_coeffs;
      double agree_rate = polyfitter.sample_once(white_point_list, temp_coeffs, number_points, 0.25);
      if (agree_rate > best_agree_rate)
      {
        best_agree_rate = agree_rate;
        best_coeffs = temp_coeffs;
      }
    }
    if (best_agree_rate > 0)
    {
      _white_lane.update(best_coeffs, _white_weight);
      _white_lane_init = true;
      _white_weight = 0.5;
    }

    else
      _white_weight *= 0.8;
  }
  void update_yellow_lane(const std::vector<cv::Point2d> yellow_point_list, int number_points = ORDER + 15,
                          int max_iter = 40)
  {
    if (!_white_lane_init)
    {
      return;
    }
    double best_agree_rate = 0;
    Eigen::VectorXd best_coeffs = _yellow_lane._coefficients;
    std::vector<cv::Point2d> points_on_right;
    for (auto point : yellow_point_list)
    {
      double white_y_here = np::polyeval(_white_lane._coefficients, point.x);
      if (point.y < white_y_here)
      {
        points_on_right.push_back(point);
      }
    }
    number_points = (number_points > points_on_right.size()) ? points_on_right.size() : number_points;
    if (points_on_right.size() < ORDER + 1)
    {
      _yellow_weight *= 0.8;
      return;
    }
    else
    {
      for (int i = 0; i < max_iter; i++)
      {
        Eigen::VectorXd temp_coeffs;
        double agree_rate = polyfitter.sample_once(points_on_right, temp_coeffs, number_points, 0.25);
        if (agree_rate > best_agree_rate)
        {
          best_agree_rate = agree_rate;
          best_coeffs = temp_coeffs;
        }
      }
      if (best_agree_rate > 0)
      {
        _yellow_lane.update(best_coeffs, _yellow_weight);
        _yellow_lane_init = true;
        _yellow_weight = 0.5;
      }

      else
        _yellow_weight *= 0.8;
    }
  }

  RANSACMachine polyfitter;
  lane _white_lane;
  lane _yellow_lane;
  double _yellow_weight;
  double _white_weight;
  Eigen::VectorXd target_lane;
  bool _white_lane_init;
  bool _yellow_lane_init;
};