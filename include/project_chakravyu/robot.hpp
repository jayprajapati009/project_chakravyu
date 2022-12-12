/**
 * @file robot.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
#define INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <utility>

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;
const double PI = 3.14159265359;

class Robot{
 private:
  int robot_id;
  ODOM current_pose;
  double theta
  RCL_NODE_PTR node_;
  ODOM goal;
  TWIST message;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::Subscription<ODOM>::SharedPtr subscriber_;

 public:
  Robot(RCL_NODE_PTR node, int robot_id);
  void set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z);
  void publish();
  void subscribe_callback(const ODOM& msg);
  double get_goal_theta(const ODOM& current_pose, const ODOM& goal);
  void move(const ODOM& current_pose, const ODOM& goal);
  double rotate(double current_theta, double goal_theta);
  void set_goal(double a, double b);
};
#endif  // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
