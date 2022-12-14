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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>



class Robot : public rclcpp::Node
{
public:
    Robot(std::string node_name,
                  std::string robot_name,
                  bool go_to_goal = false,
                  double linear_speed = 1.0,
                  double angular_speed = 1.0) : Node(node_name),
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
                                                // m_distance_to_goal{1.0}, 
                                                // m_goal_set{false}
    {
        auto current_location = std::make_pair<double, double>(3.0, 0.0);
        m_location = current_location;
        m_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto command_topic_name = "/" + m_robot_name + "/cmd_vel";
        auto pose_topic_name = "/" + m_robot_name + "/odom";

        RCLCPP_INFO_STREAM(this->get_logger(), "Robot Constructor");
        m_publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(command_topic_name, 10);
        m_goal_reached_publisher = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);
        m_subscriber_robot3_pose = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic_name, 10, std::bind(&Robot::robot_pose_callback, this, std::placeholders::_1));
        // Call on_timer function 5 times per second
        m_go_to_goal_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / 1)), std::bind(&Robot::go_to_goal_callback, this), m_cbg);
    }

    // /**
    //  * @brief Move a robot to a goal position.
    //  */
    // void go_to_goal();

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
        RCLCPP_INFO_STREAM(this->get_logger(), "Going to goal: [" << m_goal_x << "," << m_goal_y << "]");
    }

    void stop();

private:
    // attributes
    std::string m_robot_name;
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
    // bool m_goal_set;
    rclcpp::CallbackGroup::SharedPtr m_cbg;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher_cmd_vel;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_goal_reached_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_robot3_pose;
    std::pair<double, double> m_location;
    geometry_msgs::msg::Quaternion m_orientation;
    rclcpp::TimerBase::SharedPtr m_go_to_goal_timer;

    /**
     * @brief Compute the distance between two points.
     *
     * @param a The first point.
     * @param b The second point.
     * @return double   The distance between a and b.
     */
    double compute_distance(const std::pair<double, double> &a, const std::pair<double, double> &b);

    /**
     * @brief Callback function for the robot3 pose.
     *
     * @param msg Odometry message.
     */
    void robot_pose_callback(const nav_msgs::msg::Odometry& msg);
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

    void go_to_goal_callback();

};
#endif  // INCLUDE_PROJECT_CHAKRAVYU_ROBOT_HPP_
