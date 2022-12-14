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


Master::Master(std::vector<std::shared_ptr<Robot>> const &robot_array):Node("master_node") {
// creates publisher to publish /cmd_vel topic
// auto pubTopicName = "cmd_vel";
// this->publisher_ = this->create_publisher<TWIST> (pubTopicName, 10);
// create a 10Hz timer for processing
this->robot_array=robot_array;
auto processCallback = std::bind(&Master::process_callback, this);
this->timer_ = this->create_wall_timer(100ms, processCallback);
this->circle(10.0);
}

void Master::process_callback() {
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
    
}


void Master::circle(double radius) {
    int n=10;
    double h = 2*3.142/n;
    int id = 0;
    for (double i = 0.0 ; i < h*n ; i += h) {
        double a = radius * cos(i);
        double b = radius * sin(i);
        this->robot_array[id]->set_goal(a, b);
        RCLCPP_INFO_STREAM(this->get_logger(), "robot_id "<<id<<" a "<<a<<" b "<<b);
        id+=1;

    }
}
