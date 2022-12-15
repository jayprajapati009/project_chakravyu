#include <iostream>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

class TestingPublisher : public testing::Test
{
protected:
    rclcpp::Node::SharedPtr test_node_;
};

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