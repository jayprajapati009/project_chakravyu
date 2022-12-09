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
#include "project_chakravyu/robot.hpp"
using TWIST = geometry_msgs::msg::Twist;

class Master: public rclcpp::Node {
 private:
    std::vector<Robot> robot_array;
    std::vector<rclcpp::Publisher<TWIST>::SharedPtr> publisher_array;
     ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
    // rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
    // rclcpp::Publisher<TWIST>::SharedPtr    publisher_;
    rclcpp::TimerBase::SharedPtr           timer_;

 public:
    Master();
    void process_callback();
    void add_robot(int robot_id);
    // publish
    // get_pose
    // set_location
    // algorithm

    // ~Master();
};


#endif  // INCLUDE_PROJECT_CHAKRAVYU_MASTER_HPP_
