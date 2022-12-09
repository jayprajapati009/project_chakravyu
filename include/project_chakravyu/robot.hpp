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
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


class Robot : public rclcpp::Node {
 private:
  geometry_msgs::msg::Twist message = geometry_msgs::msg::Twist();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    publisher_;
  rclcpp::TimerBase::SharedPtr           timer_;
 

 public:
  Robot();
  void publish();
  void set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z);
  ~Robot();
};
#endif  // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
