#!/usr/bin/env python3
"""
The simulated Bota: MuJoCo's wrist force and torque sensors (panda_ft_force / panda_ft_torque in
franka_description/mujoco/franka/panda_ft.xml, published by mujoco_ros2_sensors as Vector3Stamped at every physics
step) republished as the Bota driver's WrenchStamped on /bota_ft_sensor/wrench, in the sensor's frame, so the
force-law controller reads the simulation exactly as it reads the robot.

    python3 sim_bota.py            (started by launch/sim/franka_sim_law.launch.py)
"""
import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from rclpy.node import Node

FORCE, TORQUE, OUT = "panda_ft_force", "panda_ft_torque", "/bota_ft_sensor/wrench"


class SimBota(Node):
    def __init__(self, name="sim_bota"):
        super().__init__(name)
        self.pub = self.create_publisher(WrenchStamped, OUT, 10)
        self.relaying = True  # False: publish nothing (a dead sensor), the force still read
        self.force = self.torque = None
        self.subs = []
        self.find = self.create_timer(0.5, self.find_topics)  # the sensors plugin's topics carry its node's namespace

    def find_topics(self):
        topics = [t for t, _ in self.get_topic_names_and_types()]
        f = [t for t in topics if t.endswith("/" + FORCE) or t == FORCE]
        q = [t for t in topics if t.endswith("/" + TORQUE) or t == TORQUE]
        if not f or not q:
            self.get_logger().info(f"waiting for {FORCE} / {TORQUE}", throttle_duration_sec=5.0)
            return
        self.find.cancel()
        self.subs = [self.create_subscription(Vector3Stamped, q[0], self.on_torque, 10),
                     self.create_subscription(Vector3Stamped, f[0], self.on_force, 10)]
        self.get_logger().info(f"relaying {f[0]} + {q[0]} -> {OUT}")

    def on_torque(self, msg):
        self.torque = msg.vector

    def on_force(self, msg):
        self.force = msg.vector
        if not self.relaying or self.torque is None:
            return
        out = WrenchStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "panda_ft_site"
        out.wrench.force = msg.vector
        out.wrench.torque = self.torque
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimBota()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
