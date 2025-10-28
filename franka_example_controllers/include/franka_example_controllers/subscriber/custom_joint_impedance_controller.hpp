#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

#include <Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * <Custom Joint Impedance controller>
 */
class CustomJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  int num_joints;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d dq_;
  rclcpp::Time start_time_;
  void updateJointStates();

  Vector7d q_desired_;
  Vector7d dq_desired_;
  Vector7d tau_desired_;

  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d torque_limit_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_joint_;
  void desiredJointCallback(const sensor_msgs::msg::JointState& msg);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace franka_example_controllers
