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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using std::placeholders::_1;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;
using ODOM = nav_msgs::msg::Odometry;

/*
 * @brief Construct a new Camera:: Camera object
 *
 */
Robot::Robot(RCL_NODE_PTR node, int robot_id):node_(node), robot_id(robot_id) {
    this->robot_id = robot_id;
    auto pubTopicName = "robot_"+std::to_string(robot_id)+ "/cmd_vel";
    this->publisher_ = node_->create_publisher<geometry_msgs::msg::Twist> (pubTopicName, 10);
    auto subTopicName = "robot_"+std::to_string(robot_id)+"/odom";
    auto subCallback = std::bind(&Robot::subscribe_callback, this, _1);
    this->subscriber_ = node_->create_subscription<ODOM> (subTopicName, 10, subCallback);
}
void Robot::publish() {
    if (current_theta == goal_theta) {
        this->move(current_pose, goal_pose);
    } else {
        this->rotate(curent_theta, goal_theta);
    }
    this->set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
void Robot::subscribe_callback(const ODOM& msg) {
    this->current_pose = msg;
    // tf2::Quaternion tf2_quat, tf2_quat_from_msg;
    // tf2::convert(this->current_pose.pose.pose.orientation, tf2_quat_from_msg);
    // // double theta=tf2::getYaw(tf2_quat_from_msg);
    RCLCPP_INFO(node_->get_logger(), "Sub Called %f", theta);
  }

double Robot::get_goal_theta(const ODOM& current_pose, const ODOM& goal) {
    double x1 = current_pose.pose.pose.position.x;
    double y1 = current_pose.pose.pose.position.y;
    double x2 = goal.pose.pose.position.x;
    double y2 = goal.pose.pose.position.x;

    double h = sqrt(pow(y2-y1, 2.0));
    double w = sqrt(pow(x2-x1, 2.0));
    return atan(h/w);
}

double Robot::rotate(double current_theta, double goal_theta) {
    while (current_theta < goal_theta) {
        this->set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 4.0);
        this->publish();
    }
    this->set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    this->publish();
}

void Robot::move(const ODOM& current_pose, const ODOM& goal) {
    double x1 = current_pose.pose.pose.position.x;
    double y1 = current_pose.pose.pose.position.y;
    double x2 = goal.pose.pose.position.x;
    double y2 = goal.pose.pose.position.x;
    while (x1 < x2 && y1 < y2) {
         this->set_vel(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        this->publish();
    }
    this->set_vel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    this->publish();
}

void Robot::set_goal(double a, double b) {
    this->goal.pose.pose.position.x = a;
    this->goal.pose.pose.position.y = b;
}
