/**
 * @file cleaner2.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "project_chakravyu/master.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;





/**
 * @brief main function entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Master>());
  rclcpp::shutdown();
  return 0;
}
