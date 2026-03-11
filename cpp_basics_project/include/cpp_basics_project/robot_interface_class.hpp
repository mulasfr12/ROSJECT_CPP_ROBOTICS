#pragma once

#ifndef ROBOT_INTERFACE_CLASS_HPP
#define ROBOT_INTERFACE_CLASS_HPP

// rclcpp includes
#include "rclcpp/callback_group.hpp"                    // callback groups
#include "rclcpp/executors.hpp"                         // executors
#include "rclcpp/executors/multi_threaded_executor.hpp" // multithreaded executor
#include "rclcpp/logger.hpp"                            // logger
#include "rclcpp/node.hpp"                              // base node
#include "rclcpp/publisher.hpp"                         // publisher
#include "rclcpp/qos.hpp"                               // qos
#include "rclcpp/rclcpp.hpp"                            // rclcpp
#include "rclcpp/subscription.hpp"                      // subscriber
#include "rclcpp/subscription_options.hpp"              // subscriber options
#include "rclcpp/timer.hpp"                             // wall timer
#include "rclcpp/utilities.hpp"                         // utilities
#include "rmw/qos_profiles.h"                           // qos profiles
// ros2 interface includes
#include "geometry_msgs/msg/detail/twist__struct.hpp" // twist message structure
#include "geometry_msgs/msg/twist.hpp"                // twist message
#include "nav_msgs/msg/detail/odometry__struct.hpp" // odometry message structure
#include "nav_msgs/msg/odometry.hpp"                // odometry message
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp" // laser scan message structure
#include "sensor_msgs/msg/laser_scan.hpp"                // laser scan message
// fastbot does not have an imu
// #include "sensor_msgs/msg/detail/imu__struct.hpp"   // imu message structure
// #include "sensor_msgs/msg/imu.hpp"                  // imu message
// standard cpp includes
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

// define robot interface class as a subclass of node class
class RobotInterface : public rclcpp::Node {
public:
  // class constructor
  RobotInterface();

  // class destructor
  ~RobotInterface();

  // shorten lengthy class references
  using Twist = geometry_msgs::msg::Twist;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Odometry = nav_msgs::msg::Odometry;
  // fastbot does not have an imu
  // using Imu = sensor_msgs::msg::Imu;

  // define and initialize class variables
  const float pi = 3.141592654;
  const float pi_inv = 0.318309886;
  double lin_vel_min = 0.000;
  double lin_vel_max = 0.000;
  double ang_vel_min = 0.000;
  double ang_vel_max = 0.000;
  // cmd_vel publisher variables
  Twist twist_cmd;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  // scan subscriber variables
  LaserScan scan_msg;
  double scan_angle_min = 0.0;
  double scan_angle_max = 0.0;
  double scan_angle_increment = 0.0;
  double scan_range_min = 0.0;
  double scan_range_max = 0.0;
  std::vector<double> scan_ranges;
  // odom subscriber variables
  Odometry odom_msg;
  double odom_position_x = 0.0;
  double odom_position_y = 0.0;
  double odom_position_z = 0.0;
  double odom_orientation_r = 0.0;
  double odom_orientation_p = 0.0;
  double odom_orientation_y = 0.0;
  std::map<std::string, double> angles;
  // fastbot does not have an imu
  // imu subscriber variables
  // Imu imu_msg;
  // double imu_angular_velocity_x = 0.0;
  // double imu_angular_velocity_y = 0.0;
  // double imu_angular_velocity_z = 0.0;
  // double imu_linear_acceleration_x = 0.0;
  // double imu_linear_acceleration_y = 0.0;
  // double imu_linear_acceleration_z = 0.0;

  // other process variables
  bool is_sim = false;
  std::string env_type = "";

private:
  // define callbacks and callback group
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  rclcpp::SubscriptionOptions scan_sub_options;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub;
  rclcpp::SubscriptionOptions odom_sub_options;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  // fastbot does not have an imu
  // rclcpp::SubscriptionOptions imu_sub_options;
  // rclcpp::Subscription<Imu>::SharedPtr imu_sub;
  rclcpp::TimerBase::SharedPtr control_timer;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  // class methods and callbacks
  void scan_callback(const LaserScan::SharedPtr scan_msg);
  void odom_callback(const Odometry::SharedPtr odom_msg);
  // fastbot does not have an imu
  // void imu_callback(const Imu::SharedPtr imu_msg);
  void control_callback();
  void publish_twist_cmd();
  std::map<std::string, double> euler_from_quaternion(double quat_x,
                                                      double quat_y,
                                                      double quat_z,
                                                      double quat_w);
};

#endif // ROBOT_INTERFACE_CLASS_HPP

// End of Code
