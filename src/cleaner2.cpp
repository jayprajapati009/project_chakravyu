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

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using IMAGE = sensor_msgs::msg::Image;
using TWIST = geometry_msgs::msg::Twist;

typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;


class RoomBa : public rclcpp::Node {
 public:
  RoomBa() :
    Node("walker"){
    // creates publisher to publish /cmd_vel topic
    publisher_1 = this->create_publisher<TWIST> ("robot_1/cmd_vel", 10);
    publisher_2 = this->create_publisher<TWIST> ("robot_2/cmd_vel", 10);
    publisher_3 = this->create_publisher<TWIST> ("robot_3/cmd_vel", 10);
    publisher_4 = this->create_publisher<TWIST> ("robot_4/cmd_vel", 10);
    publisher_5 = this->create_publisher<TWIST> ("robot_5/cmd_vel", 10);
    publisher_6 = this->create_publisher<TWIST> ("robot_6/cmd_vel", 10);
    publisher_7 = this->create_publisher<TWIST> ("robot_7/cmd_vel", 10);
    publisher_8 = this->create_publisher<TWIST> ("robot_8/cmd_vel", 10);
    publisher_9 = this->create_publisher<TWIST> ("robot_9/cmd_vel", 10);
    publisher_0 = this->create_publisher<TWIST> ("robot_0/cmd_vel", 10);




    // creates subscriber to get /demo_cam/mycamera/depth_demo topic
    // auto subTopicName = "robot_1/mycamera/depth_demo";
    // auto subCallback = std::bind(&RoomBa::subscribe_callback, this, _1);
    // subscription_ = this->create_subscription<IMAGE>
    //         (subTopicName, 10, subCallback);

    // create a 10Hz timer for processing
    auto processCallback = std::bind(&RoomBa::process_callback, this);
    timer_ = this->create_wall_timer(100ms, processCallback);
  }

 private:
  // int turns = 0;
  // void subscribe_callback(const IMAGE& msg) {
  //   lastImg_ = msg;
  // }
  /**
   * @brief Function to handle timer callback for publisher node
   * 
   */
  void process_callback() {
    // Do nothing until the first data read
    auto message = TWIST();
    message.angular.z = 4.0;
    publisher_1->publish(message);
    publisher_2->publish(message);
    publisher_3->publish(message);
    publisher_4->publish(message);
    publisher_5->publish(message);
    publisher_6->publish(message);
    publisher_7->publish(message);
    publisher_8->publish(message);
    publisher_9->publish(message);
    publisher_0->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "State = FORWARD");
  }
  ////////////////////////////////////////
  // member variables
  ////////////////////////////////////////
  rclcpp::Subscription<IMAGE>::SharedPtr subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_1;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_2;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_3;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_4;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_5;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_6;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_7;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_8;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_9;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_0;
  rclcpp::TimerBase::SharedPtr           timer_;
};

/**
 * @brief main function entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}
