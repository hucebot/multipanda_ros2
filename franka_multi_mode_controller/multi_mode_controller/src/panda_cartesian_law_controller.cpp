#include <multi_mode_controller/controllers/panda_cartesian_law_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>
#include <multi_mode_controller/utils/damping_design.h>
#include <multi_mode_controller/utils/nullspace_projection.h>

#include <cmath>
#include <limits>

using namespace panda_controllers;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Controller = PandaCartesianLawController;

static auto registration = ControllerFactory::registerClass<Controller>("panda_cartesian_law_controller");

bool Controller::initImpl(const std::vector<RobotData*>&, rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                          std::string name, std::string) {
  // where the force comes from and how the sensor is mounted in the end-effector frame (O_T_EE; roll, pitch, yaw in rad)
  const std::string prefix = name + ".";
  if (!node->has_parameter(prefix + "force_topic")) node->declare_parameter(prefix + "force_topic", force_topic_);
  if (!node->has_parameter(prefix + "force_timeout")) node->declare_parameter(prefix + "force_timeout", force_timeout_);
  if (!node->has_parameter(prefix + "sensor_rpy"))
    node->declare_parameter(prefix + "sensor_rpy", std::vector<double>{0.0, 0.0, 0.0});
  if (!node->has_parameter(prefix + "force_sign")) node->declare_parameter(prefix + "force_sign", 1.0);
  force_topic_ = node->get_parameter(prefix + "force_topic").as_string();
  force_timeout_ = node->get_parameter(prefix + "force_timeout").as_double();
  const auto rpy = node->get_parameter(prefix + "sensor_rpy").as_double_array();
  // +1: the sensor reports the force the flange exerts on the hand (the law's convention, MuJoCo's force sensor); -1:
  // the opposite (a mirror image, which no mount rotation can express). Measured by ros2/tools/ft_calibration.py.
  force_sign_ = node->get_parameter(prefix + "force_sign").as_double() < 0.0 ? -1.0 : 1.0;
  if (rpy.size() == 3) {
    sensor_rotation_ = (Eigen::AngleAxisd(rpy[2], Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(rpy[0], Vector3d::UnitX())).toRotationMatrix();
  }
  return true;
}

void Controller::startROSComImpl() {
  force_sub_ = PandaControllerInterface::node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_topic_, rclcpp::SensorDataQoS(), std::bind(&Controller::forceCallback, this, std::placeholders::_1));
  RCLCPP_INFO(PandaControllerInterface::node_->get_logger(),
              "panda_cartesian_law_controller: force from %s (timeout %.3f s, sign %+.0f)", force_topic_.c_str(),
              force_timeout_, force_sign_);
}

void Controller::forceCallback(const geometry_msgs::msg::WrenchStamped& msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  force_ = force_sign_ * Vector3d(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
  force_stamp_ = PandaControllerInterface::node_->get_clock()->now();
  has_force_ = true;
}

void Controller::startImpl() {
  law_.setParams(getParametersBuffered().law);
  law_.reset();  // tares at its first step with a real reading: start the controller with nothing touching the hand
  law_started_ = false;
  tare_raw_.setZero();
  first_tick_ = true;
  tau_prev_.setZero();
}

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau, const Pose& desired,
                                const Params& p) {
  const Pose current = getCurrentPose();
  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  const Eigen::Matrix3d ee_rotation = current.orientation.toRotationMatrix();

  // the wrist force, in the sensor's frame. The law tares at its first step, so it starts only with a real reading.
  // With no fresh force (none yet, or stale) the law is reset every tick and fed "nothing felt" (the tare reading, not
  // zero: zero minus the tare would look like the hand's weight pushing): plain impedance, no push (the law alone would
  // hold its push while nothing moves), the torque rate limit smoothing the drop; when the force is back the law
  // starts again from the same tare.
  if (retare_requested_.exchange(false)) {  // SetForceLaw retare: tare again with the next reading
    law_started_ = false;
    law_.reset();
  }
  Vector3d f_raw = tare_raw_;
  {
    std::lock_guard<std::mutex> lock(force_mutex_);
    const bool fresh = has_force_ &&
        (PandaControllerInterface::node_->get_clock()->now() - force_stamp_).seconds() < force_timeout_;
    if (fresh) {
      f_raw = force_;
      if (!law_started_) {
        law_.reset();
        tare_raw_ = force_;
        law_started_ = true;
      }
    } else {
      law_.reset();
      RCLCPP_WARN_THROTTLE(PandaControllerInterface::node_->get_logger(), *PandaControllerInterface::node_->get_clock(),
                           1000, "panda_cartesian_law_controller: no fresh force on %s, nothing felt (no push)",
                           force_topic_.c_str());
    }
  }
  law_.setParams(p.law);
  Vector3d target, k, z;
  law_.step(desired.position, current.position, f_raw, ee_rotation * sensor_rotation_, CONTROL_PERIOD, target, k, z);

  Matrix6d stiffness = Matrix6d::Zero();
  stiffness.diagonal() << k, Vector3d::Constant(p.rot_stiffness);
  Vector6d ratio;
  ratio << z, Vector3d::Constant(p.rot_damping_ratio);

  Vector6d error;
  error.head(3) << current.position - target;
  Quaterniond q_cur = current.orientation;
  if (desired.orientation.coeffs().dot(q_cur.coeffs()) < 0.0) q_cur.coeffs() << -q_cur.coeffs();
  Eigen::AngleAxisd rot_error(Quaterniond(q_cur * desired.orientation.inverse()));
  error.tail(3) << rot_error.axis() * rot_error.angle();

  const Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia), stiffness, ratio);
  Vector7d tau_task = jacobian.transpose() * (-stiffness * error - D * (jacobian * qD));
  Vector7d tau_nullspace = getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (p.nullspace_stiffness * (desired.q_n - current.q_n) - (2.0 * std::sqrt(p.nullspace_stiffness)) * qD);
  Vector7d tau_d = tau_task + tau_nullspace + coriolis;
  if (!tau_d.allFinite()) {  // never a non-finite torque: Coriolis only (the robot holds itself against gravity)
    RCLCPP_ERROR_THROTTLE(PandaControllerInterface::node_->get_logger(), *PandaControllerInterface::node_->get_clock(),
                          1000, "panda_cartesian_law_controller: non-finite torque, commanding Coriolis only");
    tau_d = coriolis;
  }

  // safety: the torque changes at most MAX_TORQUE_RATE per tick (the first tick starts from the gravity-free command)
  if (!first_tick_) {
    tau_d = tau_prev_ + (tau_d - tau_prev_).cwiseMax(-MAX_TORQUE_RATE).cwiseMin(MAX_TORQUE_RATE);
  }
  tau_prev_ = tau_d;
  first_tick_ = false;
  for (size_t i = 0; i < 7; ++i) (*tau[0])[i] = tau_d[i];
}

Controller::Params Controller::defaultParameters() {
  Params p;  // the law of ForceVAM's datasets: 1500 N/m, damping ratio 1, lead 0.5, push 40 N/s up to 60 N
  p.law.k_rest = p.law.k_load = 1500.0;
  p.law.zeta_rest = p.law.zeta_load = 1.0;
  p.law.lead = 0.5;
  p.law.tau = 0.03;
  p.law.push = 40.0;
  p.law.push_max = 60.0;
  p.law.k_max = 3000.0;
  p.law.zeta_max = 1.0;
  return p;
}

Controller::Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_data_[0]->state().O_T_EE.data()));
  p.position = transform.translation();
  p.orientation = Quaterniond(transform.linear());
  p.q_n = q;
  return p;
}

bool Controller::desiredPoseCallbackImpl(Pose& p_d, const Pose& p, const GoalMsg& msg) {
  p_d.position = Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  const Eigen::Vector4d q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                          msg.pose.orientation.z);
  if (!p_d.position.allFinite() || !q.allFinite() || std::abs(q.norm() - 1.0) > 0.01 ||
      !Eigen::Map<const Vector7d>(msg.q_n.data()).allFinite()) {
    RCLCPP_ERROR_THROTTLE(PandaControllerInterface::node_->get_logger(), *PandaControllerInterface::node_->get_clock(),
                          1000, "panda_cartesian_law_controller: discarding a planned pose that is not finite or whose "
                          "quaternion is not unit (norm %.3f)", q.norm());
    return false;
  }
  if ((p_d.position - p.position).norm() > 0.1) {
    RCLCPP_WARN_THROTTLE(PandaControllerInterface::node_->get_logger(), *PandaControllerInterface::node_->get_clock(),
                         1000, "panda_cartesian_law_controller: discarding a planned pose %.3f m away (max 0.1 m)",
                         (p_d.position - p.position).norm());
    return false;
  }
  p_d.orientation = Quaterniond(q[0], q[1], q[2], q[3]).normalized();
  if (p.orientation.angularDistance(p_d.orientation) > 0.15) {
    RCLCPP_WARN_THROTTLE(PandaControllerInterface::node_->get_logger(), *PandaControllerInterface::node_->get_clock(),
                         1000, "panda_cartesian_law_controller: discarding a planned pose rotated %.3f rad (max 0.15)",
                         p.orientation.angularDistance(p_d.orientation));
    return false;
  }
  p_d.q_n = Eigen::Map<const Vector7d>(msg.q_n.data());
  return true;
}

bool Controller::setParametersCallbackImpl(Params& p_d, const Params&, const Service::Request::SharedPtr& req,
                                           const Service::Response::SharedPtr& res) {
  if (req->k_max > 3000.0 || req->k_rest <= 0.0 || req->k_load <= 0.0 || req->tau <= 0.0 || req->zeta_max <= 0.0) {
    res->success = false;
    res->message = "rejected: k_max above 3000 N/m, or a non-positive stiffness, tau or damping cap";
    return false;
  }
  p_d.law.k_rest = req->k_rest;
  p_d.law.k_load = req->k_load;
  p_d.law.zeta_rest = req->zeta_rest;
  p_d.law.zeta_load = req->zeta_load;
  p_d.law.lead = req->lead;
  p_d.law.give = req->give;
  p_d.law.guard = req->guard > 0.0 ? req->guard : std::numeric_limits<double>::infinity();
  p_d.law.tau = req->tau;
  p_d.law.push = req->push;
  p_d.law.push_max = req->push_max;
  p_d.law.k_max = req->k_max;
  p_d.law.zeta_max = req->zeta_max;
  p_d.law.tank = req->tank;
  p_d.rot_stiffness = req->rot_stiffness;
  p_d.rot_damping_ratio = req->rot_damping_ratio;
  p_d.nullspace_stiffness = req->nullspace_stiffness;
  if (req->retare) retare_requested_ = true;
  res->success = true;
  res->message = req->retare ? "set, re-taring" : "set";
  return true;
}
