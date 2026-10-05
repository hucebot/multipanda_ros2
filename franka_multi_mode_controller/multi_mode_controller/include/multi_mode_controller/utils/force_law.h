#pragma once
// The force law that closes the force loop inside the controller: a line-by-line port of ForceVAM's control/law.py
// (LawCore), so the robot runs exactly what the policies were trained with in simulation. Checked against the Python
// law on recorded traces (test/force_law_test.cpp, traces from ForceVAM's control/export_law_traces.py).
//
// From the planned target (sent by a planner at ~50 Hz), every controller tick it sets the commanded target, the
// stiffness and the damping ratio per axis:
//  - force: the wrist reading rotated to the world frame, zeroed at the first step (tare) and low-pass filtered (tau);
//  - target: hand + lead x (planned - hand), held while the force is above the guard, plus the admittance shift (give),
//    plus the PUSH: while the hand is held back along the commanded direction, a pull growing at `push` N/s up to
//    `push_max` N, let go within PUSH_RELEASE once the hand moves faster than PUSH_SPEED ("pull until it gives");
//  - stiffness and damping ratio per axis from the resistance on it: k_rest -> k_load and zeta_rest -> zeta_load at
//    LOAD_FORCE, within the caps (k_max, zeta_max);
//  - optional energy tank on stiffness increases (passivity).
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace panda_controllers {

struct ForceLawParams {
  double k_rest = 1500.0, k_load = 1500.0;      // N/m
  double zeta_rest = 1.0, zeta_load = 1.0;
  double lead = 0.5;
  double give = 0.0;                            // mm/s per N beyond DEADBAND
  double guard = std::numeric_limits<double>::infinity();  // N
  double tau = 0.05;                            // s
  double push = 0.0, push_max = 60.0;           // N/s, N
  double k_max = 3000.0, zeta_max = 8.0;
  bool tank = false;
  double tank_init = 0.5, tank_max = 1.0;       // J
};

class ForceLaw {
 public:
  static constexpr double LOAD_FORCE = 20.0;    // N
  static constexpr double DIRECTION = 0.001;    // m
  static constexpr double DEADBAND = 2.0;       // N
  static constexpr double MAX_OFFSET = 0.1;     // m
  static constexpr double NOMINAL_MASS = 3.0;   // kg
  static constexpr double STIFF_MIN = 50.0, ZETA_MIN = 0.1;
  static constexpr double PUSH_SPEED = 0.005;   // m/s
  static constexpr double PUSH_RELEASE = 0.03;  // s

  using V3 = Eigen::Vector3d;

  explicit ForceLaw(const ForceLawParams& p = ForceLawParams()) : p_(p) { reset(); }

  void setParams(const ForceLawParams& p) { p_ = p; }
  const ForceLawParams& params() const { return p_; }

  // a new episode: tare again, forget the reactions' state
  void reset() {
    has_f0_ = false;
    f0_.setZero();
    force_.setZero();
    shift_.setZero();
    push_ = 0.0;
    has_push_dir_ = false;
    push_dir_.setZero();
    has_last_ = false;
    last_target_.setZero();
    last_hand_.setZero();
    k_.setConstant(std::numeric_limits<double>::quiet_NaN());
    zeta_.setConstant(std::numeric_limits<double>::quiet_NaN());
    energy_ = p_.tank_init;
    guarded_ = false;
  }

  // per axis, the force pushing back against the commanded motion (N, >= 0)
  static V3 resistance(const V3& force, const V3& offset) {
    V3 r;
    for (int i = 0; i < 3; ++i) {
      r[i] = std::max(force[i] * offset[i] / std::max(std::abs(offset[i]), DIRECTION), 0.0);
    }
    return r;
  }

  void impedance(const V3& force, const V3& offset, V3& k, V3& z) const {
    const V3 f = resistance(force, offset) / LOAD_FORCE;
    for (int i = 0; i < 3; ++i) {
      k[i] = std::clamp(p_.k_rest + (p_.k_load - p_.k_rest) * f[i], STIFF_MIN, p_.k_max);
      z[i] = std::clamp(p_.zeta_rest + (p_.zeta_load - p_.zeta_rest) * f[i], ZETA_MIN, p_.zeta_max);
    }
  }

  // planned: target from the planner; hand: EE position; f_raw: wrist force (sensor frame); rot: sensor-to-world
  // rotation; dt: time since the last step (s). Writes the commanded target, stiffness and damping ratio.
  void step(const V3& planned, const V3& hand, const V3& f_raw, const Eigen::Matrix3d& rot, double dt,
            V3& target, V3& k, V3& z) {
    const double a = dt / (p_.tau + dt);
    if (!has_f0_) {
      f0_ = f_raw;
      has_f0_ = true;
    }
    force_ = (1.0 - a) * force_ + a * (rot * (f_raw - f0_));
    const V3 f = force_;
    V3 base = hand + p_.lead * (planned - hand);
    guarded_ = f.norm() > p_.guard;
    if (guarded_ && has_last_) base = last_target_;
    if (p_.give > 0.0) {
      const double excess = std::max(f.norm() - DEADBAND, 0.0);
      if (excess > 0.0) {
        shift_ = shift_ - p_.give * 1e-3 * excess * f / f.norm() * dt;
        shift_ = shift_.cwiseMax(-MAX_OFFSET).cwiseMin(MAX_OFFSET);
      }
    }
    target = base + shift_;
    V3 push_offset = V3::Zero();
    if (p_.push > 0.0) {
      const V3 offset = target - hand;
      const double n = offset.norm();
      if (n > DIRECTION) {
        const V3 u = offset / n;
        if (has_push_dir_ && u.dot(push_dir_) < 0.0) push_ = 0.0;  // the planner turned back
        push_dir_ = u;
        has_push_dir_ = true;
        const double speed = has_last_ ? (hand - last_hand_).dot(u) / dt : 0.0;
        if (speed > PUSH_SPEED) {
          push_ *= std::exp(-dt / PUSH_RELEASE);  // it gave: let go
        } else if (f.dot(u) > DEADBAND) {
          push_ = std::min(push_ + p_.push * dt, p_.push_max);  // held back: pull harder
        }
        const V3 k_prev = k_.hasNaN() ? V3::Constant(p_.k_rest) : k_;
        const double k_u = std::max(k_prev.cwiseProduct(u.cwiseProduct(u)).sum(), STIFF_MIN);
        push_offset = u * push_ / k_u;
      } else {
        push_ *= std::exp(-dt / PUSH_RELEASE);
      }
      target = target + push_offset;
    }
    const V3 offset = target - hand;
    if (offset.norm() > MAX_OFFSET) target = hand + offset * MAX_OFFSET / offset.norm();
    impedance(f, target - hand, k, z);
    if (p_.tank && !k_.hasNaN()) k = tank(k, target, hand, dt);
    k_ = k;
    zeta_ = z;
    last_target_ = target - shift_ - push_offset;
    last_hand_ = hand;
    has_last_ = true;
  }

  const V3& force() const { return force_; }
  double pushForce() const { return push_; }
  bool guarded() const { return guarded_; }
  double energy() const { return energy_; }

 private:
  // refilled by what the damping dissipated since the last step, drained by the spring energy a stiffness increase
  // adds (1/2 dk e^2); increases are scaled down when the tank cannot pay for them
  V3 tank(const V3& k, const V3& target, const V3& hand, double dt) {
    const V3 v = (hand - last_hand_) / dt;
    const V3 d = 2.0 * zeta_.cwiseProduct((NOMINAL_MASS * k_).cwiseSqrt());
    energy_ = std::min(energy_ + d.cwiseProduct(v.cwiseProduct(v)).sum() * dt, p_.tank_max);
    const V3 e = target - hand;
    V3 dk = (k - k_).cwiseMax(0.0);
    double cost = 0.5 * dk.cwiseProduct(e.cwiseProduct(e)).sum();
    if (cost > energy_ && cost > 0.0) {
      dk = dk * energy_ / cost;
      cost = energy_;
    }
    energy_ -= cost;
    return k.cwiseMin(k_ + dk);
  }

  ForceLawParams p_;
  bool has_f0_ = false, has_push_dir_ = false, has_last_ = false, guarded_ = false;
  V3 f0_, force_, shift_, push_dir_, last_target_, last_hand_, k_, zeta_;
  double push_ = 0.0, energy_ = 0.5;
};

}  // namespace panda_controllers
