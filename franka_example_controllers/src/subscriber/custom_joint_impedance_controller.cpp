#include <franka_example_controllers/subscriber/custom_joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CustomJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
CustomJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type CustomJointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  // Compute PD control torques
  // tau = Kp * (q_desired - q) + Kd * (dq_desired - dq)

  tau_desired_ = k_gains_.cwiseProduct(q_desired_ - q_) + d_gains_.cwiseProduct(dq_desired_ - dq_);

  // Apply torque limits for safety
  for (int i = 0; i < num_joints; ++i) {
    tau_desired_(i) = std::clamp(tau_desired_(i), -torque_limit_(i), torque_limit_(i));
  }

  // Send computed torques to the robot
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_desired_(i));
  }

  return controller_interface::return_type::OK;
  return controller_interface::return_type::OK;
}

CallbackReturn CustomJointImpedanceController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("num_joints", 7);
    auto_declare<std::string>("topic", "/joint_impedance/joints_desired");
    auto_declare<std::vector<double>>("k_gains", std::vector<double>(7, 100.0));
    auto_declare<std::vector<double>>("d_gains", std::vector<double>(7, 0.0));
    auto_declare<std::vector<double>>("torque_limits", std::vector<double>(7, 87.0));

    // Create subscription to MPC commands
    sub_desired_joint_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        get_node()->get_parameter("topic").as_string(), 10,
        std::bind(&CustomJointImpedanceController::desiredJointCallback, this,
                  std::placeholders::_1));

    param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CustomJointImpedanceController::parametersCallback, this, std::placeholders::_1)
    );

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CustomJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Load parameters
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  num_joints = get_node()->get_parameter("num_joints").as_int();

  // Load PD gains
  std::vector<double> k_gains_vec = get_node()->get_parameter("k_gains").as_double_array();
  std::vector<double> d_gains_vec = get_node()->get_parameter("d_gains").as_double_array();
  std::vector<double> torque_limits_vec =
      get_node()->get_parameter("torque_limits").as_double_array();

  // Validate parameter sizes
  if (k_gains_vec.size() != num_joints || d_gains_vec.size() != num_joints ||
      torque_limits_vec.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter vector sizes must match num_joints!");
    return CallbackReturn::ERROR;
  }

  // Initialize Eigen vectors
  k_gains_ = Eigen::Map<Eigen::VectorXd>(k_gains_vec.data(), num_joints);
  d_gains_ = Eigen::Map<Eigen::VectorXd>(d_gains_vec.data(), num_joints);
  torque_limit_ = Eigen::Map<Eigen::VectorXd>(torque_limits_vec.data(), num_joints);

  // Initialize state vectors
  q_.resize(num_joints);
  dq_.resize(num_joints);
  q_desired_.resize(num_joints);
  dq_desired_.resize(num_joints);

  // Initialize desired states to zero
  q_desired_.setZero();
  dq_desired_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CustomJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();

  // Initialize desired positions to current positions for smooth start
  q_desired_ = q_;
  dq_desired_.setZero();

  RCLCPP_INFO(get_node()->get_logger(),
              "Joint Impedance Controller activated. Waiting for commands...");

  return CallbackReturn::SUCCESS;
}

void CustomJointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void CustomJointImpedanceController::desiredJointCallback(const sensor_msgs::msg::JointState& msg) {
  // Update desired joint positions and velocities from MPC
  for (int i = 0; i < num_joints; ++i) {
    q_desired_(i) = msg.position[i];
    dq_desired_(i) = msg.velocity[i];
  }

}

rcl_interfaces::msg::SetParametersResult CustomJointImpedanceController::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  
  for (const auto& param : parameters) {
    if (param.get_name() == "k_gains") {
      std::vector<double> new_k_gains = param.as_double_array();
      if (new_k_gains.size() != num_joints) {
        result.successful = false;
        result.reason = "k_gains must have " + std::to_string(num_joints) + " elements";
        return result;
      }
      k_gains_ = Eigen::Map<Eigen::VectorXd>(new_k_gains.data(), num_joints);
      RCLCPP_INFO(get_node()->get_logger(), "Updated k_gains dynamically");
    }
    else if (param.get_name() == "d_gains") {
      std::vector<double> new_d_gains = param.as_double_array();
      if (new_d_gains.size() != num_joints) {
        result.successful = false;
        result.reason = "d_gains must have " + std::to_string(num_joints) + " elements";
        return result;
      }
      d_gains_ = Eigen::Map<Eigen::VectorXd>(new_d_gains.data(), num_joints);
      RCLCPP_INFO(get_node()->get_logger(), "Updated d_gains dynamically");
    }
    else if (param.get_name() == "torque_limits") {
      std::vector<double> new_limits = param.as_double_array();
      if (new_limits.size() != num_joints) {
        result.successful = false;
        result.reason = "torque_limits must have " + std::to_string(num_joints) + " elements";
        return result;
      }
      torque_limit_ = Eigen::Map<Eigen::VectorXd>(new_limits.data(), num_joints);
      RCLCPP_INFO(get_node()->get_logger(), "Updated torque_limits dynamically");
    }
  }
  
  return result;
}





}  // namespace franka_example_controllers




#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CustomJointImpedanceController,
                       controller_interface::ControllerInterface)
