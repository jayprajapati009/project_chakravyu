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

#ifndef LIBRARY_ROBOT_HPP_
#define LIBRARY_ROBOT_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief A class that read images or videos from the directory
 * 
 */
class Robot {
 private:
  auto message = TWIST();
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_;
  rclcpp::TimerBase::SharedPtr           timer_;
  /**
   *
   *  @Param VideoObject This store the object of the video
   *
   */

 public:
  Robot(int robot_id);
  void publish();
  void set_vel(float l_x, float l_y, float l_z, float a_x, float a_y, float a_z);
  ~Robot();
};
#endif  // LIBRARY_ROBOT_HPP_
