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
 * @file test.cpp
 * @author Jay Prajapati (jayp@umd.edu) Shantanu Parab (sparab@umd.edu)
 * @brief test cases for the module
 * @version 0.1
 * @date 2022-12-15
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "project_chakravyu/robot.hpp"
#include "project_chakravyu/master.hpp"

/**
 * @brief Testing Publisher count
 *
 */
class TestSuite : public testing::Test
{
protected:
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<Robot> robot;
  std::shared_ptr<Master> master;
};

///////////////////////////////////////////////////////////
/// Tests
///////////////////////////////////////////////////////////

TEST_F(TestSuite, test_publisher_count)
{
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub_1 =
      test_node_->create_publisher<std_msgs::msg::String>("topic", 10.0);
  auto test_pub_2 =
      test_node_->create_publisher<std_msgs::msg::String>("topic", 10.0);

  auto number_of_publishers = test_node_->count_publishers("topic");
  EXPECT_EQ(2, static_cast<int>(number_of_publishers));
}
TEST_F(TestSuite, robot_object)
{
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  robot->go_to_goal_callback();
  EXPECT_EQ(1, 1);
}
TEST_F(TestSuite, robot_testing)
{
  auto r_namespace = "robot_" + std::to_string(1);
  auto nodename = "robot_" + std::to_string(1) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);

  auto number_of_publishers = robot->count_publishers("/" + r_namespace + "/cmd_vel");
  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}
//Testing Number of Slaves Spawned Counting the publihsers
TEST_F(TestSuite, slave_spawn_testing_publishers)
{ int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array, static_cast<int>(nodes));
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_publishers = robot_array[i]->count_publishers("/" + r_namespace + "/cmd_vel");
    pub_count = pub_count+static_cast<int>(number_of_publishers);
  }
  EXPECT_EQ(nodes, pub_count);
}

//Testing Number of Slaves Spawned_Counting the subscribers
TEST_F(TestSuite, slave_spawn_testing_subscribers)
{ int nodes = 10;
  int pub_count = 0;
  rclcpp::executors::MultiThreadedExecutor exec;
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto nodename = "robot_" + std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array, static_cast<int>(nodes));
  for (int i = 0; i < nodes; i++) {
    auto r_namespace = "robot_" + std::to_string(i);
    auto number_of_subs = robot_array[i]->count_subscribers("/" + r_namespace + "/odom");
    pub_count = pub_count+static_cast<int>(number_of_subs);
  }
  EXPECT_EQ(nodes, pub_count);
}

// Check Method Compute Distance
TEST_F(TestSuite, compute_distance)
{ 
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  std::pair<double, double> goal{10.0, 10.0};
  std::pair<double, double> loc{0.0, 0.0};
  double distance = robot->compute_distance(loc, goal);
  double ex = 14.1421;
  EXPECT_NEAR(ex, distance,0.1);
}

// Check Method Nomrmalize Angle 
TEST_F(TestSuite, normalize_angle)
{ 
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  double angle = robot->normalize_angle(-10.14);
  double ex = 2.42637;
  EXPECT_NEAR(ex, angle,0.1);
}

// Check Method Nomrmalize Angle Positive
TEST_F(TestSuite, normalize_angle_positive)
{ 
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  double angle = robot->normalize_angle_positive(-7.29);
  double ex = 5.2763;
  EXPECT_NEAR(ex, angle,0.1);
}

// Check Method Yaw From Quaternions
TEST_F(TestSuite, compute_yaw_from_quaternion)
{ 
  int n = 1;
  auto r_namespace = "robot_" + std::to_string(n);
  auto nodename = "robot_" + std::to_string(n) + "_controller";
  robot = std::make_shared<Robot>(nodename, r_namespace);
  robot->m_orientation.x=3.0;
  robot->m_orientation.y=5.0;
  robot->m_orientation.z=4.0;
  robot->m_orientation.w=6.0;
  double yaw = robot->compute_yaw_from_quaternion();
  double ex = 1.51955;
  EXPECT_NEAR(ex, yaw, 0.1);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}