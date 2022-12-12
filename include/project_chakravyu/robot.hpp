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

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

class Robot{
 private:
  int robot_id;
  ODOM odometry;
  RCL_NODE_PTR node_;
  
  TWIST message;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::Subscription<ODOM>::SharedPtr subscriber_;

 public:
  Robot(RCL_NODE_PTR node, int robot_id);
  void set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z);
  void publish();
  void subscribe_callback(const ODOM& msg);
  // ~Robot();
  
};
#endif  // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
