#pragma once

#include <memory>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class GripperSubscriber : public rclcpp::Node {
 public:
  /// Constructor
  GripperSubscriber();

 private:
  // Franka gripper object (connection to hardware)
  std::unique_ptr<franka::Gripper> gripper_;

  // protect concurrent access to gripper
  std::mutex gripper_state_mutex_;

  // gripper state
  franka::GripperState current_gripper_state_;

  // gripper binary state memory
  bool command_data_bool_prev;

  // ROS2 subscription
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr command_sub_;

  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr width_pub_;

  // Timer for periodic state publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string robot_ip_;
  std::vector<std::string> joint_names_;
  double default_speed_;
  double default_width_;
  double maximum_width_;
  double gripper_max_effort_;
  double default_epsilon_inner_;
  double default_epsilon_outer_;
  int pub_frequency_;

  // Callback function executed when a message is received
  void commandCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void publishGripperState();
};
