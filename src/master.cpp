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
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include "project_chakravyu/master.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;


Master::Master():Node("master_node") {
// creates publisher to publish /cmd_vel topic
// auto pubTopicName = "cmd_vel";
// this->publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);
for(int i ; i < 10 ; i++) {
    this->add_robot(i);
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot Added");
}


// create a 10Hz timer for processing
auto processCallback = std::bind(&Master::process_callback, this);
this->timer_ = this->create_wall_timer(100ms, processCallback);
}

void Master::process_callback() {
auto message = TWIST();
message.angular.z = -0.4;
for(rclcpp::Publisher<TWIST>::SharedPtr p : this->publisher_array) {
    p->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
}
RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
}

void Master::add_robot(int robot_id) {
    // this->robot_array.push_back(Robot(robot_id));
    auto pubTopicName = "robot_"+std::to_string(robot_id)+ "/cmd_vel";
    this->publisher_array.push_back(this->create_publisher<TWIST> (pubTopicName, 10));

    auto subTopicName = "robot_"+std::to_string(robot_id)+"/odom";
    auto subCallback = std::bind(&Master::subscribe_callback, this, _1);
    this->pose_subscriber_array.push_back(this->create_subscription<ODOM> (subTopicName, 10, subCallback));
}

void Master::subscribe_callback(const ODOM& msg) {
    this->odometry = msg;
    RCLCPP_INFO_STREAM(this->get_logger(), "Subscriber Called = STOP");
  }
