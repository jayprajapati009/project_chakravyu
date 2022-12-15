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
 * @author Jay Prajapati (jayp@umd.edu)
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

/**
 * @brief Testing Publisher count
 *
 */
class TestingPublisher : public testing::Test
{
protected:
  rclcpp::Node::SharedPtr test_node_;
};

///////////////////////////////////////////////////////////
/// Tests
///////////////////////////////////////////////////////////

TEST_F(TestingPublisher, test_publisher_count)
{
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub_1 =
      test_node_->create_publisher<std_msgs::msg::String>("topic", 10.0);
  auto test_pub_2 =
      test_node_->create_publisher<std_msgs::msg::String>("topic", 10.0);

  auto number_of_publishers = test_node_->count_publishers("topic");
  EXPECT_EQ(2, static_cast<int>(number_of_publishers));
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}