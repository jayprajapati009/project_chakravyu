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
 * @file robot.hpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Library for robot.cpp
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
#define INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_

#include <string>
#include <utility>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class Robot : public rclcpp::Node
{
public:
    Robot(std::string node_name, std::string robot_name, bool go_to_goal = false,
          double linear_speed = 2.0, double angular_speed = 2.0)
        : Node(node_name),
          m_robot_name{robot_name},
          m_go_to_goal{go_to_goal},
          m_linear_speed{linear_speed},
          m_angular_speed{angular_speed},
          m_roll{0},
          m_pitch{0},
          m_yaw{0},
          m_kv{1},
          m_kh{1},
          m_goal_x{0.0},
          m_goal_y{0.0}
    {
        auto current_location = std::make_pair(3.0, 0.0);
        m_location = current_location;
        m_cbg = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto command_topic_name = "/" + m_robot_name + "/cmd_vel";
        auto pose_topic_name = "/" + m_robot_name + "/odom";

        RCLCPP_INFO_STREAM(this->get_logger(), "Robot Constructor");
        m_publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
            command_topic_name, 10);
        m_goal_reached_publisher =
            this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
        m_subscriber_robot3_pose =
            this->create_subscription<nav_msgs::msg::Odometry>(
                pose_topic_name, 10,
                std::bind(&Robot::robot_pose_callback, this,
                          std::placeholders::_1));
        // Call on_timer function 5 times per second
        m_go_to_goal_timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / 1)),
            std::bind(&Robot::go_to_goal_callback, this), m_cbg);
    }

    /**
     * @brief Set the goal to reach.
     *
     * @param go_to_goal Flag used to perform a transform listener
     * @param x x-coordinate of the goal position.
     * @param y y-coordinate of the goal position.
     */
    void set_goal(double x, double y)
    {
        m_go_to_goal = true;
        m_goal_x = x;
        m_goal_y = y;
        RCLCPP_INFO_STREAM(this->get_logger(), "Going to goal: [" << m_goal_x << ","
                                                                  << m_goal_y
                                                                  << "]");
    }
    /**
     * @brief Stop the robot from moving
     *
     */
    void stop();
    /**
     * @brief Compute the distance between two points.
     *
     * @param a The first point.
     * @param b The second point.
     * @return double   The distance between a and b.
     */
    double compute_distance(const std::pair<double, double> &a,
                            const std::pair<double, double> &b);

    /**
     * @brief Callback function for the robot3 pose.
     *
     * @param msg Odometry message.
     */
    void robot_pose_callback(const nav_msgs::msg::Odometry &msg);
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

    /**
     * @brief Compute the yaw angle from quaternion pose
     *
     * @return double
     */
    double compute_yaw_from_quaternion();

    /**
     * @brief To move to robot by publish velocities on cmd_vel
     *
     * @param linear linear velocity component
     * @param angular angular velocity component
     */
    void move(double linear, double angular);

    /**
     * @brief process to move the robot to a goal
     *
     */
    void go_to_goal_callback();
    geometry_msgs::msg::Quaternion m_orientation;

private:
    // attributes
    std::string m_robot_name; // robot name used for creating namespace
    bool m_go_to_goal;        // flag to store if the robot has reached position
    double m_linear_speed;    // base linear velocity of robot
    double m_angular_speed;   // base angular velocity of robot
    double m_roll;            // rad
    double m_pitch;           // rad
    double m_yaw;             // rad
    double m_kv;              // gain for linear velocity
    double m_kh;              // gain for angular velocity
    double m_goal_x;
    double m_goal_y;
    double m_distance_to_goal;

    rclcpp::CallbackGroup::SharedPtr m_cbg;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_goal_reached_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        m_subscriber_robot3_pose;
    std::pair<double, double> m_location;
    //   geometry_msgs::msg::Quaternion m_orientation;
    rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;
};
#endif // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
