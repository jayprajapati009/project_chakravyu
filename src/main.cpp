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
#include <stdexcept>
#include <string>
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
  int nodes = 10;
  if (argc > 0) {
        std::string arg = argv[1];
    try {
      std::size_t pos;
      nodes = std::stoi(arg, &pos);
      if (pos < arg.size()) {
        std::cerr << "Trailing characters after number: " << arg << '\n';
      }
    } catch (std::invalid_argument const &ex) {
      std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
      std::cerr << "Number out of range: " << arg << '\n';
    }
  }
  std::vector<std::shared_ptr<Robot>> robot_array;
  for (int i = 0 ; i < nodes ; i++) {
    auto r_namespace = "robot_"+std::to_string(i);
    auto nodename = "robot_"+std::to_string(i) + "_controller";
    auto robot = std::make_shared<Robot>(nodename, r_namespace);
    exec.add_node(robot);
    robot_array.push_back(robot);
  }
  auto node = std::make_shared<Master>(robot_array, static_cast<int>(nodes));
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
