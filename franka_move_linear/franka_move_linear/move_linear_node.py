#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import GripperWidth

import numpy as np


"""
To run: ros2 run franka_move_linear move_linear_node

To publish a target pose:

ros2 topic pub --once /move_linear/cartesian_pos_target geometry_msgs/PoseStamped "
pose:
  position:
    x: 0.5678
    y: 0.0035
    z: 0.5044
  orientation:
    x:  0.99886
    y: -0.00356
    z:  0.04641
    w:  0.01000
"

"""


class MoveLinear(Node):

    """
    ROS2 node that generates a Cartesian linear motion between a current and a target end-effector pose.

    The node interpolates position linearly and orientation using quaternion SLERP at a fixed control rate.
    Given a target pose, it computes a time-parameterized trajectory respecting maximum translational
    and rotational speeds, and publishes intermediate Cartesian setpoints.
    """

    def __init__(self):

        super().__init__('MoveLinear')

        # Params
        self.fps_timer = 30.0
        self.step_counter = 0
        self.n_steps = -1

        # Subscribers
   
        self.cart_pose_curr_sub = self.create_subscription(PoseStamped, '/cartesian_impedance/cartesian_pos_curr', self.cart_pose_curr_callback, 10)
        self.cart_pose_target_sub = self.create_subscription(PoseStamped, '/move_linear/cartesian_pos_target', self.cart_pose_target_callback, 10)
        
        # Publisher

        self.cart_pose_action_pub = self.create_publisher(PoseStamped, '/cartesian_impedance/equilibrium_pose', 10)
        self.gripper_action_pub = self.create_publisher(GripperWidth, '/panda_gripper/gripper_command', 10)

        # Timer according to framerate
        self.dt_timer = 1.0 / self.fps_timer
        self.timer = self.create_timer(self.dt_timer, self.move_linear_timer)

        print("MoveLinear node initialized successfully")

    def cart_pose_target_callback(self, msg):
        
        # Wait for target pose before calling 'interp_init()'
        if not hasattr(self, "cart_pos_curr"):
            print("Current pose not received yet")
            return

        # Cart pose
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Set target Cartesian pose
        self.cart_pos_target = [px, py, pz]
        self.cart_quat_target = [qx, qy, qz, qw]

        # init interpolator
        self.step_counter = 0 # refresh variable
        self.interp_init(self.cart_pos_curr, self.cart_quat_curr, self.cart_pos_target, self.cart_quat_target)

        # Publish gripper open (just one time)
        gripper_msg = GripperWidth()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        gripper_msg.width = 0.0 # OPEN

        # Publish
        self.gripper_action_pub.publish(gripper_msg)

    def cart_pose_curr_callback(self, msg):

        # Cart pose
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Set curr Cartesian pose
        self.cart_pos_curr = [px, py, pz]
        self.cart_quat_curr = [qx, qy, qz, qw]

    def calc_distance(self, p1, p2):
        
        """compute distance between two 3D vectors"""

        return np.linalg.norm(p2 - p1)
    
    def calc_rel_ang_dist_between_quats(self, q_start, q_final):

        """Compute angular distance between start and final pose"""

        # Normalize (important!)
        q_start /= np.linalg.norm(q_start)
        q_final /= np.linalg.norm(q_final)

        # Dot product between quaternions
        dot = np.dot(q_start, q_final)

        # Take shortest path (q and -q represent same rotation)
        dot = abs(dot)

        # Clamp for numerical safety
        dot = np.clip(dot, -1.0, 1.0)

        # Angular distance
        ang_dist = 2.0 * np.arccos(dot)

        return ang_dist
            
    def interp_init(self, t_start, q_start, t_final, q_final, trans_speed = 0.04, rot_speed = 0.5):
            
        """Initialize interpolation parameters"""

        # Convert lists in numpy arrays
        t_start = np.asarray(t_start, dtype=float)
        t_final = np.asarray(t_final, dtype=float)
        q_start = np.asarray(q_start, dtype=float)
        q_final = np.asarray(q_final, dtype=float)

        # linear distance
        trans_dist = self.calc_distance(t_final, t_start)
        t_trans = trans_dist / trans_speed
        # print("trans_dist: ", trans_dist)
        # print("t_trans: ", t_trans)

        # angular distance
        ang_dist = self.calc_rel_ang_dist_between_quats(q_start, q_final)
        t_rot = ang_dist / rot_speed
        # print("ang_dist: ", ang_dist)
        # print("t_rot: ", t_rot)

        # total time & corresponding n_steps
        total_time = max(t_trans, t_rot)
        self.n_steps = int(np.ceil(self.fps_timer*total_time))

        # normalize quaternions
        q_start = q_start / np.linalg.norm(q_start)
        q_final = q_final / np.linalg.norm(q_final)

        # check angle between the two quaternions
        dot = np.dot(q_start, q_final)

        # enforce shortest path for SLERP
        if np.dot(q_start, q_final) < 0.0:
            q_final = -q_final  
            dot = -dot

        # clamp to avoid NaNs
        dot = np.clip(dot, -1.0, 1.0)

        # compute angle
        self.theta = np.arccos(dot)
        self.sin_theta = np.sin(self.theta)

        # Set variables to be used during 'interp_execute()'
        self.t_start = t_start
        self.q_start = q_start
        self.t_final = t_final
        self.q_final = q_final

        # print("INTERP INIT")
        # print("t_start: ", t_start)
        # print("q_start: ", q_start)
        # print("t_final: ", t_final)
        # print("q_final: ", q_final)
        # print("INTERP INIT")
        
    def interp_execute(self, i):
        
        """Compute Cartesian pose setpoint for the current step"""

        # n_steps == 0 means Tgoal == Tinit
        # In this way I also avoid division by zero
        if self.n_steps == 0:
            s = 1.0
        else:
            s = i / self.n_steps  # compute current step

        # interpolate position
        t_interp = (1 - s) * self.t_start + s * self.t_final
        
        # interpolate orientation with SLERP (handle near-zero rotation difference where sin_theta ≈ 0)
        if np.isclose(self.sin_theta, 0.0):
            q_interp = self.q_final
        else:
            theta_t = self.theta * s
            w0 = np.sin(self.theta - theta_t) / self.sin_theta
            w1 = np.sin(theta_t) / self.sin_theta
            q_interp = w0 * self.q_start + w1 * self.q_final

        # normalize
        q_interp = q_interp / np.linalg.norm(q_interp)

        return t_interp, q_interp
    
    def move_linear_timer(self):

        """Perform linear motion"""

        # Wait at least 1 data for each buffer
        if (self.step_counter > self.n_steps):
            return

        # Current target
        t_target_interp, q_target_interp = self.interp_execute(self.step_counter)

        # Update counter
        self.step_counter +=1

        # Set cart pose action
        cart_msg = PoseStamped()
        cart_msg.header.stamp = self.get_clock().now().to_msg()
        cart_msg.pose.position.x = float(t_target_interp[0])
        cart_msg.pose.position.y = float(t_target_interp[1])
        cart_msg.pose.position.z = float(t_target_interp[2])
        cart_msg.pose.orientation.x = float(q_target_interp[0])
        cart_msg.pose.orientation.y = float(q_target_interp[1])
        cart_msg.pose.orientation.z = float(q_target_interp[2])
        cart_msg.pose.orientation.w = float(q_target_interp[3])

        # Publish
        self.cart_pose_action_pub.publish(cart_msg)

        # print("t_target_interp: ", t_target_interp)
        # print("q_target_interp: ", q_target_interp)


def main(args=None):

    rclpy.init(args=args)
    node = MoveLinear()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
