#pragma once
// panda_cartesian_law_controller: ForceVAM's force law inside the 1 kHz loop. A planner (a policy, teleoperation) sends
// the PLANNED end-effector pose (~50 Hz, <resource>/<name>/desired_pose, CartesianImpedanceGoal); every tick the law
// (utils/force_law.h, identical to ForceVAM's control/law.py) turns the planned position and the wrist force into the
// commanded position, the stiffness and the damping ratio per axis, and the torque law of the Cartesian impedance
// controller applies them (mass-aware damping, sqrtDesign; nullspace; Coriolis). Orientation: fixed impedance.
//
// Force: the Bota wrist sensor's WrenchStamped topic (ROS parameter <name>.force_topic, sensor frame), rotated to the
// base frame with the end-effector rotation (O_T_EE) and the sensor's mount rotation (<name>.sensor_rpy). The law tares it at start.
// The law starts (tares) at the first reading; a force older than <name>.force_timeout (s) then counts as nothing felt
// (the law reset every tick: plain impedance, no push), with a warning.
// Safety, as franka_example_controllers' custom Cartesian impedance controller: the torque change per tick is limited.
// Settings: service <resource>/<name>/parameters (SetForceLaw); defaults: the law ForceVAM's datasets use.
#include <Eigen/Dense>

#include <atomic>
#include <mutex>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <multi_mode_control_msgs/msg/cartesian_impedance_goal.hpp>
#include <multi_mode_control_msgs/srv/set_force_law.hpp>
#include <multi_mode_controller/base/panda_controller_ros_interface.h>
#include <multi_mode_controller/controllers/comless_panda_cartesian_impedance_controller.h>
#include <multi_mode_controller/utils/force_law.h>

namespace panda_controllers {

struct PandaCartesianLawControllerParams {
  ForceLawParams law;
  double rot_stiffness = 100.0;      // N m/rad
  double rot_damping_ratio = 1.0;
  double nullspace_stiffness = 10.0;
};

class PandaCartesianLawController :
    public virtual ControllerRosInterface<multi_mode_control_msgs::srv::SetForceLaw,
                                          multi_mode_control_msgs::msg::CartesianImpedanceGoal,
                                          PandaCartesianLawControllerParams,
                                          PandaCartesianImpedanceControllerPose> {
 public:
  using Params = PandaCartesianLawControllerParams;
  using Pose = PandaCartesianImpedanceControllerPose;
  using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
  using Service = multi_mode_control_msgs::srv::SetForceLaw;
  static constexpr double MAX_TORQUE_RATE = 1.0;   // N m per tick (1 ms)
  // one tick = 1 ms of robot time, on the robot (libfranka's 1 kHz loop) and in simulation (one update per 1 ms physics
  // step, even when the simulation runs slower than real time): the law integrates with it, not with the wall clock
  static constexpr double CONTROL_PERIOD = 0.001;  // s
  virtual ~PandaCartesianLawController() = default;

 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau, const Pose& desired,
                      const Params& p) override final;
  Params defaultParameters() override final;
  Pose getCurrentPoseImpl() override final;
  bool initImpl(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                std::string name, std::string resource) override final;
  void startImpl() override final;
  void startROSComImpl() override final;
  bool desiredPoseCallbackImpl(Pose& p_d, const Pose& p, const GoalMsg& msg) override final;
  bool setParametersCallbackImpl(Params& p_d, const Params& p, const Service::Request::SharedPtr& req,
                                 const Service::Response::SharedPtr& res) override final;
  void forceCallback(const geometry_msgs::msg::WrenchStamped& msg);

  ForceLaw law_;
  bool law_started_ = false;                      // tared with a real reading
  std::atomic<bool> retare_requested_{false};     // SetForceLaw retare: the law tares again with the next reading
  Eigen::Vector3d tare_raw_ = Eigen::Vector3d::Zero();  // that reading (sensor frame): what "nothing felt" reads
  Eigen::Matrix3d sensor_rotation_ = Eigen::Matrix3d::Identity();  // sensor frame -> end-effector frame (O_T_EE)
  std::string force_topic_ = "/bota_ft_sensor/wrench";
  double force_timeout_ = 0.1;
  double force_sign_ = 1.0;  // the sensor's sign convention (ros2/tools/ft_calibration.py in ForceVAM)
  std::mutex force_mutex_;
  Eigen::Vector3d force_ = Eigen::Vector3d::Zero();
  rclcpp::Time force_stamp_;
  std::atomic<bool> has_force_{false};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  bool first_tick_ = true;
  Eigen::Matrix<double, 7, 1> tau_prev_ = Eigen::Matrix<double, 7, 1>::Zero();
};

}  // namespace panda_controllers
