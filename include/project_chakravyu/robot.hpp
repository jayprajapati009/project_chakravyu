/**
 * @file robot.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
#define INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <cmath>

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
// #include <geometry_msgs/msg/pose2D.hpp>
// #include <tf/tf.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"



using std::placeholders::_1;
using namespace std::chrono_literals;
using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;
const double PI = 3.14159265359;

class Robot: public rclcpp::Node{
 private:
    bool m_go_to_goal;
    double m_linear_speed;
    double m_angular_speed;
    double m_roll;  // rad
    double m_pitch; // rad
    double m_yaw;   // rad
    double m_kv;    // gain for linear velocity
    double m_kh;    // gain for angular velocity
    double m_goal_x;
    double m_goal_y;
    double m_distance_to_goal;
    std::pair<double, double> m_location;
    geometry_msgs::msg::Quaternion m_orientation;
    rclcpp::Publisher<TWIST>::SharedPtr publisher_;
    rclcpp::Subscription<ODOM>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr  timer_;
    double compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);

    /**
     * @brief Callback function for the robot3 pose.
     *
     * @param msg Odometry message.
     */
    void robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    /**
     * @brief Normalizes the angle to be 0 to 2*M_PI
     *
     * @param angle Angle to normalize (rad)
     * @return double Normalized angle (rad)
     */
    double normalize_angle_positive(double angle);

    /**
     * @brief Normalizes the angle to be -M_PI circle to +M_PI circle
     *
     * @param angle Angle to normalize (rad)
     * @return double Normalized angle (rad)
     */
    double normalize_angle(double angle);
    double compute_yaw_from_quaternion();
    void move(double linear, double angular);
    void transform_callback();
    void go_to_goal_callback();
    void move_in_circle_callback();
    void subscribe_callback(const ODOM& msg);
    void process_callback();


 public:
  Robot(bool go_to_goal = false,
            double linear_speed = 0.5,
            double angular_speed = 0.3) : Node("slave"), m_go_to_goal{go_to_goal},
                                                m_linear_speed{linear_speed},
                                                m_angular_speed{angular_speed},
                                                m_roll{0},
                                                m_pitch{0},
                                                m_yaw{0},
                                                m_kv{1},
                                                m_kh{1},
                                                m_goal_x{0.0},
                                                m_goal_y{0.0}{
    auto processCallback = std::bind(&Robot::process_callback, this);
    this->timer_ = this->create_wall_timer(100ms, processCallback);
    auto pubTopicName = "cmd_vel";
    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist> (pubTopicName, 10);
    auto subTopicName = "odom";
    auto subCallback = std::bind(&Robot::subscribe_callback, this, _1);
    this->subscriber_ = this->create_subscription<ODOM> (subTopicName, 10, subCallback);
}
  void set_goal(double x, double y);
  void stop();
  // void set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z);
  // void algorithm();
  
  // double get_goal_theta();
  // void move();
  // double rotate();
  // void set_goal(double a, double b);
};
#endif  // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
