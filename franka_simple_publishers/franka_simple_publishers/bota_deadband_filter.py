#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
from scipy import signal

class WrenchFilterNode(Node):
    def __init__(self):
        super().__init__('wrench_filter_node')
        
        # Deadband Parameters
        self.declare_parameter('force_deadband', 1.0)
        self.declare_parameter('torque_deadband', 0.1)
        
        # EMA Parameter (1.0 = no filtering, 0.0 = infinite delay)
        self.declare_parameter('ema_alpha', 0.85)
        
        # Butterworth Parameters
        self.declare_parameter('butterworth_cutoff', 1.0) # Hz
        self.declare_parameter('sample_rate', 90.0) # Set this to your Bota output rate

        self.force_db = self.get_parameter('force_deadband').value
        self.torque_db = self.get_parameter('torque_deadband').value
        self.alpha = self.get_parameter('ema_alpha').value
        
        # State memory for EMA
        self.ema_force = None
        self.ema_torque = None
        
        # Setup 2nd-order Butterworth Low-Pass Filter
        cutoff = self.get_parameter('butterworth_cutoff').value
        fs = self.get_parameter('sample_rate').value
        self.sos = signal.butter(2, cutoff, btype='low', fs=fs, output='sos')
        
        # State memory for Butterworth (3 axes for force, 3 for torque)
        self.bw_force_state = [signal.sosfilt_zi(self.sos)] * 3
        self.bw_torque_state = [signal.sosfilt_zi(self.sos)] * 3

        in_topic = self.declare_parameter('input_topic', '/bota_ft_sensor/wrench').value
        out_topic = self.declare_parameter('output_topic', '/bota_ft_sensor_wrench_filtered').value

        self.subscription = self.create_subscription(WrenchStamped, in_topic, self.wrench_callback, 10)
        self.publisher = self.create_publisher(WrenchStamped, out_topic, 10)

    def apply_deadband(self, value, threshold):
        return 0.0 if abs(value) < threshold else value

    def wrench_callback(self, msg):
        raw_force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        raw_torque = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        
        bw_force = [0.0] * 3
        bw_torque = [0.0] * 3
        
        # # 1. Butterworth Filter 
        for i in range(3):
            bw_f, self.bw_force_state[i] = signal.sosfilt(self.sos, [raw_force[i]], zi=self.bw_force_state[i])
            bw_t, self.bw_torque_state[i] = signal.sosfilt(self.sos, [raw_torque[i]], zi=self.bw_torque_state[i])
            bw_force[i] = bw_f[0]
            bw_torque[i] = bw_t[0]
            
        # 2. EMA Filter
        if self.ema_force is None:
            self.ema_force = bw_force
            self.ema_torque = bw_torque
        else:
            self.ema_force = [self.alpha * bw_force[i] + (1 - self.alpha) * self.ema_force[i] for i in range(3)]
            self.ema_torque = [self.alpha * bw_torque[i] + (1 - self.alpha) * self.ema_torque[i] for i in range(3)]

        # 3. Deadband
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        
        # filtered_msg.wrench.force.x = self.ema_force[0]
        # filtered_msg.wrench.force.y = self.ema_force[1]
        # filtered_msg.wrench.force.z = self.ema_force[2]
        
        # filtered_msg.wrench.torque.x = self.ema_torque[0]
        # filtered_msg.wrench.torque.y = self.ema_torque[1]
        # filtered_msg.wrench.torque.z = self.ema_torque[2]

        filtered_msg.wrench.force.x = self.apply_deadband(self.ema_force[0], self.force_db)
        filtered_msg.wrench.force.y = self.apply_deadband(self.ema_force[1], self.force_db)
        filtered_msg.wrench.force.z = self.apply_deadband(self.ema_force[2], self.force_db)
        
        filtered_msg.wrench.torque.x = self.apply_deadband(self.ema_torque[0], self.torque_db)
        filtered_msg.wrench.torque.y = self.apply_deadband(self.ema_torque[1], self.torque_db)
        filtered_msg.wrench.torque.z = self.apply_deadband(self.ema_torque[2], self.torque_db)
        
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WrenchFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()