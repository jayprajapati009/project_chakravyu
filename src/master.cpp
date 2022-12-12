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
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;
const double PI = 3.14159265359;

Master::Master():Node("master_node") {
// creates publisher to publish /cmd_vel topic
// auto pubTopicName = "cmd_vel";
// this->publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);
for(int i = 0; i < 10 ; i++) {
    this->add_robot(i);
    RCLCPP_INFO_STREAM(this->get_logger(), "Robot Added");
}


// create a 10Hz timer for processing
auto processCallback = std::bind(&Master::process_callback, this);
this->timer_ = this->create_wall_timer(100ms, processCallback);
}

void Master::process_callback() {
    for (int i = 0; i < 10; i++) {
        robot_array_[i]->set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 4.4);
        robot_array_[i]->publish();
        RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "State = STOP");
}

void Master::add_robot(int robot_id) {
    this->robot_array_.push_back(std::make_shared<Robot>(static_cast<RCL_NODE_PTR>(this), robot_id));
}

void Master::circle(double radius) {
    double h = 2*PI/10;
    int id = 0;
    for (double i = 0.0 ; i < h ; i += h) {
        double a = radius * cos(i);
        double b = radius * sin(i);
        robot_array_[id]->set_goal(a, b);
        id+=1;
    }
}