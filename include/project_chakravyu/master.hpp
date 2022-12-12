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
using ROBOT_PTR = std::shared_ptr<Robot>;
// std::shared_ptr<CallbackFunction>

class Master: public rclcpp::Node {
 private:
    int numNodes = 5;
    std::vector<ROBOT_PTR> robot_array_  = std::vector<ROBOT_PTR>(numNodes);
    //  std::vector<rclcpp::Publisher<TWIST>::SharedPtr> publisher_array;
    //  std::vector<rclcpp::Subscription<ODOM>::SharedPtr> pose_subscriber_array;
     ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
    // rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
    // rclcpp::Publisher<TWIST>::SharedPtr    publisher_;
    rclcpp::TimerBase::SharedPtr  timer_;
    

 public:
    Master();
    void process_callback();
    void add_robot(int robot_id);
    // void subscribe_callback(const ODOM& msg);
    // publish
    // get_pose
    // set_location
    // algorithm

    // ~Master();
};


#endif  // INCLUDE_PROJECT_CHAKRAVYU_MASTER_HPP_
    