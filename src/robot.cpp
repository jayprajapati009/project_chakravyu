/**
 * @file robot.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "project_chakravyu/robot.hpp"
#include "project_chakravyu/master.hpp"
#include <rclcpp/rclcpp.hpp>

// #include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;
using ODOM = nav_msgs::msg::Odometry;

/*
 * @brief Construct a new Camera:: Camera object
 *
 */
Robot::Robot(RCL_NODE_PTR node, int robot_id):node_(node),robot_id(robot_id){
    this->robot_id = robot_id;
    auto pubTopicName = "robot_"+std::to_string(robot_id)+ "/cmd_vel";
    this->publisher_ = node_->create_publisher<geometry_msgs::msg::Twist> (pubTopicName, 10);
    // auto subTopicName = "robot_"+std::to_string(robot_id)+"/odom";
    // auto subCallback = std::bind(this->subscribe_callback, node_, _1);
    // this->subscriber_ = node_->create_subscription<ODOM> (subTopicName, 10, subCallback);
}
void Robot::publish() {
    this->publisher_->publish(this->message);
}
void Robot::set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z) {
    this->message.linear.x = l_x;
    this->message.linear.y = l_y;
    this->message.linear.z = l_z;
    this->message.angular.x = a_x;
    this->message.angular.y = a_y;
    this->message.angular.z = a_z;
}
// void Robot::subscribe_callback(const ODOM& msg) {
//     this->odometry = msg;
//     RCLCPP_INFO_STREAM(node_->get_logger(), "Sub Called");
//   }


// Robot::~Robot() {}
