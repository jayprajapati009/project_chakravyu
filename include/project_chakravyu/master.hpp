/**
 * @file master.hpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-12-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef INCLUDE_PROJECT_CHAKRAVYU_MASTER_HPP_
#define INCLUDE_PROJECT_CHAKRAVYU_MASTER_HPP_
#include <vector>
#include <memory>
#include "project_chakravyu/robot.hpp"
#include <nav_msgs/msg/odometry.hpp>

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;

class Master: public rclcpp::Node {
 private:
    rclcpp::TimerBase::SharedPtr  timer_;
    rclcpp::Publisher<TWIST>::SharedPtr publisher_; // Change to publish to custom robot array
    std::vector<std::shared_ptr<Robot>> robot_array;

 public:
    Master(std::vector<std::shared_ptr<Robot>> const &robot_array);
    void process_callback();
    void circle(double radius);
};


#endif  // INCLUDE_PROJECT_CHAKRAVYU_MASTER_HPP_
    