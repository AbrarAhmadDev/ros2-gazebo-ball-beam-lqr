#!/usr/bin/env python3
"""
1. Uses an angle-command LQR, because your ROS controller is:
      position_controllers/JointGroupPositionController
   Therefore /beam_position_controller/commands expects desired beam angle [rad],
   not torque and not beam angular acceleration.
2. Uses a 2-state outer-loop model:
      state = [ball_position, ball_velocity]
      input = theta_cmd = desired beam angle
3. Adds velocity smoothing, command clipping, command slew-rate limiting, and deadband.
4. Logs simulation data to:
      ~/ros2_ws/ball_beam_log.csv
   This CSV can be used to generate report plots:
      x(t), x_dot(t), theta_cmd(t), tracking error, phase portrait, metrics.
"""

import csv
import os

import numpy as np
from scipy.linalg import solve_continuous_are

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def compute_angle_lqr_gain():
    """
    Compute continuous-time LQR gain for the simplified ball-beam outer loop.

    Model:
        x_dot = v
        v_dot = accel_sign * beta * theta_cmd

    State:
        X = [x, v]^T

    Input:
        theta_cmd = desired beam angle [rad]

    For a sliding/prismatic ball approximation:
        beta = g

    For an ideal rolling solid sphere:
        beta = (5/7)g

    URDF currently uses a prismatic ball_joint, so beta = g
    simulation approximation.
    """
    g = 9.81
    beta = g

    ACCEL_SIGN = +1.0

    A = np.array([
        [0.0, 1.0],
        [0.0, 0.0],
    ])

    B = np.array([
        [0.0],
        [ACCEL_SIGN * beta],
    ])

    # Conservative LQR weights.
    # Increase Q[,] for faster centering.
    # Increase R for smoother and smaller beam movement.
    Q = np.diag([8.0, 1.0])
    R = np.array([[15.0]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    return K.flatten(), A, B, Q, R


K_LQR, A_SYS, B_SYS, Q_LQR, R_LQR = compute_angle_lqr_gain()


class BallBeamAngleLQR(Node):
    """
    ROS 2 node for outer-loop ball position stabilization.

    Subscribes:
        /joint_states

    Publishes:
        /beam_position_controller/commands

    Logs:
        ~/ros2_ws/ball_beam_log.csv
    """

    # Velocity smoothing factor.
    # Higher = faster velocity response but more noise.
    # Lower = smoother but slower.
    VEL_ALPHA = 0.25

    # Joint safety limits.
    # URDF/joint limit is around ±0.4 rad, so use slightly below that.
    MAX_ANGLE = 0.25       # rad.
    MAX_RATE = 1         # rad/s. Limits command jumps.

    # Deadband around equilibrium to prevent tiny oscillations.
    POS_DEADBAND = 0.001   # m
    VEL_DEADBAND = 0.008   # m/s

    def __init__(self):
        super().__init__('ball_beam_angle_lqr')

        self.get_logger().info('Modified angle-command LQR controller started.')
        self.get_logger().info(f'LQR gain K = {K_LQR}')
        self.get_logger().info(f'A matrix:\n{A_SYS}')
        self.get_logger().info(f'B matrix: {B_SYS.flatten()}')
        self.get_logger().info(f'Q matrix:\n{Q_LQR}')
        self.get_logger().info(f'R matrix:\n{R_LQR}')

        # Desired ball position: center of beam.
        self.setpoint = 0.0

        # Previous state cache.
        self.prev_time = None
        self.prev_ball_pos = 0.0
        self.prev_beam_ang = 0.0

        # Smoothed velocity estimates.
        self.smooth_ball_vel = 0.0
        self.smooth_beam_vel = 0.0

        # Previous command for slew-rate limiting.
        self.prev_cmd = 0.0

        # Publisher to beam position controller.
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/beam_position_controller/commands',
            10
        )

        # Subscriber to joint states.
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # CSV logging for report plots.
        self.log_path = os.path.expanduser('~/ros2_ws/ball_beam_log.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            'time',
            'ball_pos',
            'ball_vel',
            'beam_ang',
            'beam_vel',
            'theta_des',
            'theta_cmd',
            'tracking_error'
        ])

        self.start_time = self.get_clock().now()

        self.get_logger().info(f'Logging data to: {self.log_path}')

    def joint_state_callback(self, msg: JointState):
        now = self.get_clock().now()

        # Map joint names to indices.
        joint_index = {name: i for i, name in enumerate(msg.name)}

        if 'ball_joint' not in joint_index:
            return

        ball_i = joint_index['ball_joint']
        raw_ball_pos = float(msg.position[ball_i])

        # beam_joint is optional for logging. Controller only needs ball state.
        has_beam = 'beam_joint' in joint_index
        if has_beam:
            beam_i = joint_index['beam_joint']
            raw_beam_ang = float(msg.position[beam_i])
        else:
            beam_i = None
            raw_beam_ang = 0.0

        # First message initializes finite-difference memory.
        if self.prev_time is None:
            self.prev_time = now
            self.prev_ball_pos = raw_ball_pos
            self.prev_beam_ang = raw_beam_ang
            return

        dt = (now.nanoseconds - self.prev_time.nanoseconds) / 1e9

        # Avoid duplicate or invalid timestamps.
        if dt < 1e-4:
            return

        # Use JointState velocity if available; otherwise finite difference.
        if msg.velocity and len(msg.velocity) > ball_i:
            raw_ball_vel = float(msg.velocity[ball_i])
        else:
            raw_ball_vel = (raw_ball_pos - self.prev_ball_pos) / dt

        if has_beam and msg.velocity and len(msg.velocity) > beam_i:
            raw_beam_vel = float(msg.velocity[beam_i])
        elif has_beam:
            raw_beam_vel = (raw_beam_ang - self.prev_beam_ang) / dt
        else:
            raw_beam_vel = 0.0

        # Smooth velocities to reduce oscillations caused by noisy derivative estimates.
        self.smooth_ball_vel = (
            (1.0 - self.VEL_ALPHA) * self.smooth_ball_vel
            + self.VEL_ALPHA * raw_ball_vel
        )

        self.smooth_beam_vel = (
            (1.0 - self.VEL_ALPHA) * self.smooth_beam_vel
            + self.VEL_ALPHA * raw_beam_vel
        )

        # State error for LQR.
        tracking_error = raw_ball_pos - self.setpoint

        error_state = np.array([
            tracking_error,
            self.smooth_ball_vel,
        ])

        # Deadband near center to avoid small jitter.
        if abs(tracking_error) < self.POS_DEADBAND and abs(self.smooth_ball_vel) < self.VEL_DEADBAND:
            theta_des = 0.0
        else:
            # Correct LQR law:
            #   theta_cmd = -K * [position_error, velocity]^T
            theta_des = -float(K_LQR @ error_state)

        # Clip desired angle to safe joint command.
        theta_des = float(np.clip(theta_des, -self.MAX_ANGLE, self.MAX_ANGLE))

        # Slew-rate limit: prevents instant jumps in position command.
        max_step = self.MAX_RATE * dt
        theta_cmd = self.prev_cmd + float(np.clip(theta_des - self.prev_cmd, -max_step, max_step))

        # Publish beam angle command.
        cmd = Float64MultiArray()
        cmd.data = [theta_cmd]
        self.pub.publish(cmd)

        # Log to CSV for plotting.
        t = (now.nanoseconds - self.start_time.nanoseconds) / 1e9
        self.csv_writer.writerow([
            t,
            raw_ball_pos,
            self.smooth_ball_vel,
            raw_beam_ang,
            self.smooth_beam_vel,
            theta_des,
            theta_cmd,
            tracking_error,
        ])
        self.log_file.flush()

        # Update memory.
        self.prev_cmd = theta_cmd
        self.prev_ball_pos = raw_ball_pos
        self.prev_beam_ang = raw_beam_ang
        self.prev_time = now

        self.get_logger().info(
            f't={t:6.2f}  '
            f'ball_pos={raw_ball_pos:+.3f}  '
            f'ball_vel={self.smooth_ball_vel:+.3f}  '
            f'beam_ang={raw_beam_ang:+.3f}  '
            f'theta_des={theta_des:+.4f}  '
            f'theta_cmd={theta_cmd:+.4f}'
        )

    def destroy_node(self):
        # Close CSV cleanly when the node shuts down.
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BallBeamAngleLQR()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
