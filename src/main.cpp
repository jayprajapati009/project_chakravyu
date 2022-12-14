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
#include "project_chakravyu/master.hpp"
#include "project_chakravyu/robot.hpp"





/**
 * @brief main function entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto bot_controller = std::make_shared<Robot>("bot_controller_robot", "robot_1");
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0 ; i < 10 ; i++) {
    auto r_namespace = "robot_"+std::to_string(i);
    auto nodename = "robot_"+std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
