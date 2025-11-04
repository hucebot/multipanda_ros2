#!/usr/bin/env python3
"""
Standalone Joint Impedance Controller Tuner
Run directly without ROS2 package installation:
    python3 tune_joint_impedance.py --joint 3 --freq 0.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import argparse
import sys


class JointImpedanceTuner(Node):
    """
    Node to tune joint impedance controller by publishing sinusoidal trajectories.
    
    Publishes to: /joint_impedance/joints_desired
    """
    
    # Panda robot joint limits (in radians)
    JOINT_LIMITS = {
        'min': np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
        'max': np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    }
    
    def __init__(self, joint_to_tune=0, frequency=0.5, publish_rate=100.0, 
                 rest_positions=None, amplitude_scale=0.8):
        super().__init__('joint_impedance_tuner')
        
        # Set parameters from arguments
        self.joint_to_tune = joint_to_tune
        self.frequency = frequency
        self.publish_rate = publish_rate
        self.rest_positions = np.array(rest_positions if rest_positions is not None 
                                       else [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.amplitude_scale = amplitude_scale
        
        # Validate joint index
        if self.joint_to_tune < 0 or self.joint_to_tune > 6:
            self.get_logger().error(f'Invalid joint index: {self.joint_to_tune}. Must be 0-6.')
            raise ValueError('Invalid joint index')
        
        # Validate rest positions are within limits
        if not self._validate_positions(self.rest_positions):
            self.get_logger().warn('Some rest positions are outside joint limits. Clamping...')
            self.rest_positions = np.clip(
                self.rest_positions,
                self.JOINT_LIMITS['min'],
                self.JOINT_LIMITS['max']
            )
        
        # Calculate sinusoid parameters for the active joint
        self.center = self.rest_positions[self.joint_to_tune]
        
        # Calculate maximum safe amplitude (considering both limits)
        max_amp_pos = self.JOINT_LIMITS['max'][self.joint_to_tune] - self.center
        max_amp_neg = self.center - self.JOINT_LIMITS['min'][self.joint_to_tune]
        self.amplitude = min(max_amp_pos, max_amp_neg) * self.amplitude_scale
        
        # Angular frequency
        self.omega = 2 * np.pi * self.frequency
        
        # Publisher
        self.publisher = self.create_publisher(
            JointState,
            '/joint_impedance/joints_desired',
            10
        )
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Time tracking
        self.start_time = self.get_clock().now()
        
        # Log configuration
        self.get_logger().info('=' * 60)
        self.get_logger().info('Joint Impedance Controller Tuner')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Joint to tune: {self.joint_to_tune}')
        self.get_logger().info(f'Frequency: {self.frequency} Hz')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Center position: {self.center:.4f} rad')
        self.get_logger().info(f'Amplitude: {self.amplitude:.4f} rad')
        self.get_logger().info(f'Position range: [{self.center - self.amplitude:.4f}, '
                             f'{self.center + self.amplitude:.4f}] rad')
        self.get_logger().info(f'Joint limits: [{self.JOINT_LIMITS["min"][self.joint_to_tune]:.4f}, '
                             f'{self.JOINT_LIMITS["max"][self.joint_to_tune]:.4f}] rad')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Publishing started...')
    
    def _validate_positions(self, positions: np.ndarray) -> bool:
        """Check if positions are within joint limits."""
        return np.all(positions >= self.JOINT_LIMITS['min']) and \
               np.all(positions <= self.JOINT_LIMITS['max'])
    
    def timer_callback(self):
        """Publish joint state command at regular intervals."""
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Calculate sinusoidal position and velocity for active joint
        position = self.center + self.amplitude * np.sin(self.omega * elapsed)
        velocity = self.amplitude * self.omega * np.cos(self.omega * elapsed)
        
        # Create joint state message
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.name = [f'panda_joint{i+1}' for i in range(7)]
        
        # Set positions (sinusoid for active joint, rest for others)
        msg.position = self.rest_positions.copy().tolist()
        msg.position[self.joint_to_tune] = position
        
        # Set velocities (sinusoid derivative for active joint, zero for others)
        msg.velocity = [0.0] * 7
        msg.velocity[self.joint_to_tune] = velocity
        
        # Publish
        self.publisher.publish(msg)
        
        # Log periodically (every 2 seconds)
        if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.01:
            self.get_logger().info(
                f'Joint {self.joint_to_tune}: pos={position:.4f} rad, vel={velocity:.4f} rad/s'
            )


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='Tune joint impedance controller with sinusoidal trajectories',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Tune joint 3 (elbow) at 0.5 Hz
  python3 tune_joint_impedance.py --joint 3 --freq 0.5
  
  # Tune joint 1 with smaller amplitude
  python3 tune_joint_impedance.py --joint 1 --freq 0.3 --amp 0.6
  
  # High-speed test on joint 6
  python3 tune_joint_impedance.py --joint 6 --freq 1.5 --rate 200
        """
    )
    
    parser.add_argument('--joint', '-j', type=int, default=0,
                       help='Joint index to tune (0-6), default=0')
    parser.add_argument('--freq', '-f', type=float, default=0.5,
                       help='Sinusoid frequency in Hz, default=0.5')
    parser.add_argument('--rate', '-r', type=float, default=100.0,
                       help='Publishing rate in Hz, default=100.0')
    parser.add_argument('--amp', '-a', type=float, default=0.8,
                       help='Amplitude scale (0-1), default=0.8')
    parser.add_argument('--rest', nargs=7, type=float, default=None,
                       help='Rest positions for 7 joints (rad), default=[0, -0.785, 0, -2.356, 0, 1.571, 0.785]')
    
    # Parse args, but separate ROS args from our args
    if args is None:
        args = sys.argv[1:]
    
    # Filter out ROS arguments
    filtered_args = [arg for arg in args if not arg.startswith('__') and not arg.startswith('--ros-args')]
    cmd_args = parser.parse_args(filtered_args)
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        tuner = JointImpedanceTuner(
            joint_to_tune=cmd_args.joint,
            frequency=cmd_args.freq,
            publish_rate=cmd_args.rate,
            rest_positions=cmd_args.rest,
            amplitude_scale=cmd_args.amp
        )
        
        print("\nPress Ctrl+C to stop\n")
        rclpy.spin(tuner)
        
    except KeyboardInterrupt:
        print('\n\nTuner stopped by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()