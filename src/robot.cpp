/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Jay Prajapati, Shail Shah and Shantanu Parab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

void Robot::robot_pose_callback(const nav_msgs::msg::Odometry &msg) {
  m_location.first = msg.pose.pose.position.x;
  m_location.second = msg.pose.pose.position.y;
  m_orientation = msg.pose.pose.orientation;
}

double Robot::normalize_angle_positive(double angle) {
  const double result = fmod(angle, 2.0 * M_PI);
  if (result < 0) return result + 2.0 * M_PI;
  return result;
}

double Robot::normalize_angle(double angle) {
  const double result = fmod(angle + M_PI, 2.0 * M_PI);
  if (result <= 0.0) return result + M_PI;
  return result - M_PI;
}

double Robot::compute_distance(const std::pair<double, double> &a,
                               const std::pair<double, double> &b) {
  return sqrt(pow(b.first - a.first, 2) + pow(b.second - a.second, 2));
}

double Robot::compute_yaw_from_quaternion() {
  tf2::Quaternion q(m_orientation.x, m_orientation.y, m_orientation.z,
                    m_orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

void Robot::move(double linear, double angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  m_publisher_cmd_vel->publish(msg);
}

void Robot::stop() {
  m_go_to_goal = false;
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0;
  cmd_vel_msg.angular.z = 0;
  m_publisher_cmd_vel->publish(cmd_vel_msg);

  std_msgs::msg::Bool goal_reached_msg;
  goal_reached_msg.data = true;
  m_goal_reached_publisher->publish(goal_reached_msg);
}

void Robot::go_to_goal_callback() {
  if (!m_go_to_goal) return;

  std::pair<double, double> goal{m_goal_x, m_goal_y};
  double distance_to_goal = compute_distance(m_location, goal);

  if (distance_to_goal > 0.1) {
    distance_to_goal = compute_distance(m_location, goal);
    double angle_to_goal =
        std::atan2(m_goal_y - m_location.second, m_goal_x - m_location.first);

    if (angle_to_goal < 0)
      // angle_to_goal = 2 * M_PI + angle_to_goal;
      angle_to_goal = normalize_angle_positive(angle_to_goal);

    // angle to rotate to face the goal
    double w = angle_to_goal - compute_yaw_from_quaternion();

    if (w > M_PI) {
      w = w - 2 * M_PI;
      // w = m_normalize_angle_positive(w);
    }

    // proportional control for linear velocity
    double linear_x = std::min(m_kv * distance_to_goal, m_linear_speed);

    // proportional control for angular velocity
    double angular_z = m_kh * w;
    if (angular_z > 0)
      angular_z = std::min(angular_z, m_angular_speed);
    else
      angular_z = std::max(angular_z, -m_angular_speed);

    move(linear_x, angular_z);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "********** Goal reached **********");
    stop();
  }
}
