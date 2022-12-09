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
#include <string>
#include "project_chakravyu/master.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;


Master::Master():Node("master_node") {
// creates publisher to publish /cmd_vel topic
// auto pubTopicName = "cmd_vel";
// this->publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);
this->add_robot(1);

// create a 10Hz timer for processing
auto processCallback = std::bind(&Master::process_callback, this);
this->timer_ = this->create_wall_timer(100ms, processCallback);
}

void Master::process_callback() {
auto message = TWIST();
this->publisher_array[0]->publish(message);
RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
}

void Master::add_robot(int robot_id) {
    // this->robot_array.push_back(Robot(robot_id));
    auto pubTopicName = "robot_"+std::to_string(robot_id)+ "cmd_vel";
    this->publisher_array.push_back(this->create_publisher<TWIST> (pubTopicName, 10));
}




