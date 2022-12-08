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
#include "../library/robot.hpp"
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>


/*
 * @brief Construct a new Camera:: Camera object
 *
 */
Robot::Robot(int robot_id) {
    auto pubTopicName = "robot_"+std::to_string(robot_id)+ "cmd_vel";
    this->publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);
}
void Robot::publish() {
    this->publisher_->publish(this->message);
}
void Robot::set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z) {
    this->message.linear.x = l_x;
    this->message.linear.y = l_x;
    this->message.linear.z = l_x;
    this->message.angular.x = a_x;
    this->message.angular.y = a_y;
    this->message.angular.z = a_z;
}

Robot::~Robot() {}
