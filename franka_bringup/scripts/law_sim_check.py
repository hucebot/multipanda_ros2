#!/usr/bin/env python3
"""
The force-law controller's check on the simulated panda, before the real one. Start the simulation without its own
relay, then this script (it relays the wrist sensor itself, to be able to cut it):

    ros2 launch franka_bringup franka_sim_law.launch.py headless:=true sim_bota:=false
    python3 law_sim_check.py --out /tmp/law_sim_check

Phases, each with a pass/fail line (planned poses sent at 50 Hz, as a policy does):
  1. rest: the wrist force with nothing touching (the hand's weight, which the law tares)
  2. free motion: 5 cm sideways; the hand ends within 5 mm of the planned position (this simulated arm stops ~4 mm
     short with the stock Cartesian impedance controller too)
  3. press: down onto the block until it is felt (3 N for 30 ms), then the planned position held 2 cm inside it for 4 s:
     the force is the hand's push on the world (negative z), "pull until it gives" makes it grow, it settles (no
     oscillation) and stays under the law's ceiling
  4. dead sensor: the relay stops for 1.5 s while pressing: the controller warns and drops to plain impedance (the
     push lets go, the force falls); then the relay comes back
  5. release: planned 5 cm up; the force returns to zero
  6. a planned pose 0.2 m away is refused (the hand does not move)
  7. settings service: k_max 4000 N/m refused, a softer law accepted, the default law set back
Writes forces.csv (1 kHz: time, wrist force in the world frame, relayed or not) and poses.csv (50 Hz: time, hand,
planned) to --out, and summary.txt with the pass/fail lines.
"""
import argparse
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from multi_mode_control_msgs.msg import CartesianImpedanceGoal
from multi_mode_control_msgs.srv import GetRobotStates, SetForceLaw
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from sim_bota import SimBota

RATE = 50.0              # Hz, planned poses
SENSOR_RPY_Z = -math.pi / 4  # the sensor site in the hand, turned from the flange (config/sim/single_sim_law.yaml)
DEFAULT_LAW = dict(k_rest=1500.0, k_load=1500.0, zeta_rest=1.0, zeta_load=1.0, lead=0.5, give=0.0, guard=0.0,
                   tau=0.03, push=40.0, push_max=60.0, k_max=3000.0, zeta_max=1.0, tank=False, rot_stiffness=100.0,
                   rot_damping_ratio=1.0, nullspace_stiffness=10.0)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_from_matrix(R):
    """(w, x, y, z), stable at every angle (Shepperd: from the largest of w, x, y, z; the hand pointing down is a
    180 deg turn, where the trace formula divides by ~0)."""
    t = np.trace(R)
    i = int(np.argmax([t, R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = 2.0 * math.sqrt(1.0 + t)
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif i == 1:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif i == 2:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4]
    q = np.array(q)
    return q / np.linalg.norm(q)


class Check(SimBota):
    def __init__(self, out):
        super().__init__("law_sim_check")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.out = out
        self.goal_pub = None
        self.state_cli = self.law_cli = None
        self.R_ee = np.eye(3)
        self.forces, self.poses, self.lines = [], [], []
        self.lock = threading.Lock()

    # the wrist force, rotated to the world frame, logged at every physics step
    def on_force(self, msg):
        super().on_force(msg)
        f = self.R_ee @ rot_z(SENSOR_RPY_Z) @ np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        with self.lock:
            self.forces.append((t, *f, float(self.relaying)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, seconds):  # simulated seconds
        t_end = self.now() + seconds
        while self.now() < t_end:
            time.sleep(0.002)

    def connect(self):
        while True:
            topics = dict(self.get_topic_names_and_types())
            goal = [t for t in topics if t.endswith("panda_cartesian_law_controller/desired_pose")]
            services = dict(self.get_service_names_and_types())
            state = [s for s in services if s.endswith("/get_robot_states")]
            law = [s for s in services if s.endswith("panda_cartesian_law_controller/parameters")]
            if goal and state and law and self.subs and self.forces:
                break
            self.get_logger().info("waiting for the controller and the sensor", throttle_duration_sec=5.0)
            time.sleep(0.5)
        self.goal_pub = self.create_publisher(CartesianImpedanceGoal, goal[0], 10)
        self.state_cli = self.create_client(GetRobotStates, state[0])
        self.law_cli = self.create_client(SetForceLaw, law[0])
        self.get_logger().info(f"goals on {goal[0]}, state from {state[0]}, settings on {law[0]}")

    def call(self, client, request):
        future = client.call_async(request)
        while not future.done():
            time.sleep(0.001)
        return future.result()

    def state(self):
        s = self.call(self.state_cli, GetRobotStates.Request()).states[0]
        T = np.array(s.o_t_ee).reshape(4, 4).T  # column-major
        self.R_ee = T[:3, :3]
        return T[:3, 3].copy(), T[:3, :3].copy(), np.array(s.q)

    def send(self, position, R, q_n):
        msg = CartesianImpedanceGoal()
        msg.pose = Pose()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
        w, x, y, z = quat_from_matrix(R)
        msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z = w, x, y, z
        msg.q_n = [float(v) for v in q_n]
        self.goal_pub.publish(msg)

    def follow(self, planned_fn, seconds, R, q_n):
        """Send planned_fn(elapsed) at RATE for `seconds` (simulated); log hand and planned; returns the last hand."""
        t0 = self.now()
        hand = None
        while self.now() - t0 < seconds:
            planned = planned_fn(self.now() - t0)
            self.send(planned, R, q_n)
            hand, _, _ = self.state()
            self.poses.append((self.now(), *hand, *planned))
            self.wait(1.0 / RATE)
        return hand

    def force_since(self, t0):
        with self.lock:
            f = np.array([r[1:4] for r in self.forces if r[0] >= t0])
        return f if len(f) else np.zeros((1, 3))

    def report(self, ok, text):
        line = f"{'PASS' if ok else 'FAIL'}  {text}"
        self.lines.append(line)
        self.get_logger().info(line)

    def run(self):
        self.connect()
        hand0, R, q_n = self.state()
        # 1. rest
        t0 = self.now()
        self.follow(lambda e: hand0, 1.0, R, q_n)
        f_rest = self.force_since(t0).mean(0)
        self.report(True, f"rest: wrist force {np.round(f_rest, 2)} N in the world frame (the hand's weight, tared by the law)")

        # 2. free motion: 5 cm along y in 1 s, hold 1.5 s
        goal = hand0 + np.array([0.0, 0.05, 0.0])
        hand = self.follow(lambda e: hand0 + (goal - hand0) * min(e / 1.0, 1.0), 2.5, R, q_n)
        err = np.linalg.norm(hand - goal)
        self.report(err < 0.005, f"free motion: 5 cm sideways, ends {1000 * err:.1f} mm from the planned position (< 5 mm)")

        # 3. press: down at 2 cm/s until 3 N, then 2 cm inside the block for 4 s
        start = hand.copy()
        t0 = self.now()
        contact = None
        while self.now() - t0 < 10.0:
            planned = start - np.array([0.0, 0.0, 0.02 * (self.now() - t0)])
            self.send(planned, R, q_n)
            hand, _, _ = self.state()
            self.poses.append((self.now(), *hand, *planned))
            # felt: 3 N on every reading of the last 30 ms (the jolt of starting to move is shorter)
            recent = self.force_since(self.now() - 0.03) - f_rest
            if self.now() - t0 > 0.5 and len(recent) > 5 and np.linalg.norm(recent, axis=1).min() > 3.0:
                contact = hand.copy()
                break
            self.wait(1.0 / RATE)
        if contact is None:
            self.report(False, "press: no contact within 10 s (20 cm)")
            return self.finish()
        self.report(True, f"press: contact at z = {contact[2]:.3f} m")
        inside = contact - np.array([0.0, 0.0, 0.02])
        t_press = self.now()
        self.follow(lambda e: inside, 4.0, R, q_n)
        f = self.force_since(t_press) - f_rest
        fz = f[:, 2]
        ts = np.linspace(0, 4.0, len(fz))
        at = {s: float(fz[min(int(s / 4.0 * len(fz)), len(fz) - 1)]) for s in (0.25, 0.5, 1.0, 2.0, 3.0, 3.9)}
        self.report(fz[-200:].mean() < -3.0, f"press: the force is the hand's push on the world (z {fz[-200:].mean():.1f} N < 0)")
        self.report(abs(at[2.0]) > abs(at[0.25]) + 5.0,
                    "press: pull until it gives, the force grows: " + ", ".join(f"{s}s {v:.1f}" for s, v in at.items()) + " N")
        last = fz[ts > 3.0]
        self.report(last.std() < 2.0, f"press: settled, no oscillation (std over the last second {last.std():.2f} N < 2)")
        self.report(np.abs(fz).max() < 90.0, f"press: peak {np.abs(fz).max():.1f} N (< 90: spring + push_max 60 N)")

        # 4. dead sensor for 1.5 s while pressing
        before = float(fz[-200:].mean())
        self.relaying = False
        t_dead = self.now()
        self.follow(lambda e: inside, 1.5, R, q_n)
        dead = (self.force_since(t_dead) - f_rest)[:, 2]
        self.relaying = True
        after = float(dead[-200:].mean())
        self.report(abs(after) < abs(before) - 5.0,
                    f"dead sensor: plain impedance, the push lets go ({before:.1f} N -> {after:.1f} N); see the controller's warning")
        self.follow(lambda e: inside, 1.0, R, q_n)

        # 5. release: 5 cm up in 1 s, hold 1 s
        up = inside + np.array([0.0, 0.0, 0.05])
        t_up = self.now()
        hand = self.follow(lambda e: inside + (up - inside) * min(e / 1.0, 1.0), 2.0, R, q_n)
        f_up = float(np.linalg.norm((self.force_since(t_up + 1.5) - f_rest).mean(0)))
        self.report(f_up < 2.0, f"release: force back to {f_up:.2f} N (< 2)")

        # 6. a planned pose 0.2 m away is refused
        far = hand + np.array([0.2, 0.0, 0.0])
        before_far = hand.copy()
        hand = self.follow(lambda e: far, 0.5, R, q_n)
        moved = np.linalg.norm(hand - before_far)
        self.report(moved < 0.002, f"far pose refused: the hand moved {1000 * moved:.1f} mm (< 2)")

        # 7. settings service
        req = SetForceLaw.Request(**dict(DEFAULT_LAW, k_max=4000.0))
        res = self.call(self.law_cli, req)
        self.report(not res.success, f"settings: k_max 4000 N/m refused ({res.message})")
        res = self.call(self.law_cli, SetForceLaw.Request(**dict(DEFAULT_LAW, k_rest=800.0, k_load=800.0)))
        self.report(res.success, f"settings: a softer law (800 N/m) accepted ({res.message})")
        self.follow(lambda e: hand, 0.5, R, q_n)
        res = self.call(self.law_cli, SetForceLaw.Request(**DEFAULT_LAW))
        self.report(res.success, "settings: the default law set back")
        return self.finish()

    def finish(self):
        os.makedirs(self.out, exist_ok=True)
        with self.lock:
            np.savetxt(os.path.join(self.out, "forces.csv"), np.array(self.forces), delimiter=",",
                       header="t,fx,fy,fz,relayed", comments="")
        np.savetxt(os.path.join(self.out, "poses.csv"), np.array(self.poses), delimiter=",",
                   header="t,x,y,z,planned_x,planned_y,planned_z", comments="")
        with open(os.path.join(self.out, "summary.txt"), "w") as f:
            f.write("\n".join(self.lines) + "\n")
        n_fail = sum(line.startswith("FAIL") for line in self.lines)
        self.get_logger().info(f"{len(self.lines) - n_fail}/{len(self.lines)} passed; written to {self.out}")
        return n_fail == 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="/tmp/law_sim_check")
    args = p.parse_args()
    rclpy.init()
    node = Check(args.out)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    ok = node.run()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
