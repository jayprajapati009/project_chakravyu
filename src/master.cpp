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
 * @file master.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief Algorithm to run the robot
 * @version 0.1
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <string>

#include "project_chakravyu/master.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using TWIST = geometry_msgs::msg::Twist;
using ODOM = nav_msgs::msg::Odometry;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

/**
 * @brief Construct a new Master:: Master object
 * 
 * @param robot_array 
 * @param nodes 
 */
Master::Master(std::vector<std::shared_ptr<Robot>> const &robot_array,
               int nodes)
    : Node("master_node") {
  this->robot_array = robot_array;
  this->nodes = nodes;
  auto processCallback = std::bind(&Master::process_callback, this);
  this->timer_ = this->create_wall_timer(100ms, processCallback);
  this->circle(10.0);
}

/**
 * @brief Process callback logger info
 * 
 */
void Master::process_callback() {
  RCLCPP_INFO_STREAM(this->get_logger(), "iterating Publisher array");
}

/**
 * @brief Makes a circle for the spawned robots
 * 
 * @param radius 
 */
void Master::circle(double radius) {
  double h = 2 * 3.142 / this->nodes;
  int id = 0;
  for (double i = 0.0; i < h * this->nodes; i += h) {
    double a = radius * cos(i);
    double b = radius * sin(i);
    if (i < 2 * 3.14) {
      this->robot_array[id]->set_goal(a, b);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "robot_id " << id << " a " << a << " b " << b);
      id += 1;
    }
  }
}
