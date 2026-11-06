#!/usr/bin/env python3
"""Bridge auto_driving VehicleState messages to the c305 Gazebo vehicle."""

import math
from typing import Dict, Optional

from car_msg.msg import VehicleState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class C305SimAdapter(Node):
    """Translate vehicle commands and publish feedback from simulated state."""

    LEFT_STEER_JOINT = 'steer_front_left_joint'
    RIGHT_STEER_JOINT = 'steer_front_right_joint'

    def __init__(self):
        super().__init__('c305_sim_adapter')

        self.wheelbase = float(self.declare_parameter('wheelbase', 1.152).value)
        self.track_width = float(
            self.declare_parameter('track_width', 0.902).value)
        self.steer_gain = float(self.declare_parameter('steer_gain', 0.5).value)
        self.max_steering_angle = float(
            self.declare_parameter('max_steering_angle', 0.5).value)
        self.max_forward_speed = float(
            self.declare_parameter('max_forward_speed', 0.70).value)
        self.max_reverse_speed = float(
            self.declare_parameter('max_reverse_speed', 0.12).value)
        self.command_timeout = float(
            self.declare_parameter('command_timeout', 0.5).value)
        self.brake_threshold = float(
            self.declare_parameter('brake_threshold', 0.01).value)
        self.feedback_rate = float(
            self.declare_parameter('feedback_rate', 20.0).value)
        self.publish_vehicle_state = bool(
            self.declare_parameter('publish_vehicle_state', True).value)
        self.simulated_battery = float(
            self.declare_parameter('simulated_battery', 100.0).value)

        if self.wheelbase <= 0.0 or self.track_width <= 0.0:
            raise ValueError('wheelbase and track_width must be positive')
        if self.steer_gain <= 0.0 or self.max_steering_angle <= 0.0:
            raise ValueError('steer_gain and max_steering_angle must be positive')
        if self.feedback_rate <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError('feedback_rate and command_timeout must be positive')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.info_pub = self.create_publisher(
            VehicleState, '/vehicle_info', 10)
        self.state_pub = None
        if self.publish_vehicle_state:
            self.state_pub = self.create_publisher(
                VehicleState, '/vehicle_state', 10)

        self.create_subscription(
            VehicleState, '/control_cmd', self._on_control_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10)

        self.latest_velocity = 0.0
        self.joint_positions: Dict[str, float] = {}
        self.last_command_time = None
        self.brake_active = False
        self.timeout_active = False
        self.warned_nonzero_gear = False

        period = 1.0 / self.feedback_rate
        self.create_timer(period, self._publish_feedback)
        self.create_timer(min(0.05, self.command_timeout / 2.0),
                          self._command_watchdog)

        self.get_logger().info(
            'c305 simulation adapter started: '
            f'wheelbase={self.wheelbase:.3f} m, '
            f'track={self.track_width:.3f} m, '
            f'steer_gain={self.steer_gain:.3f}, '
            f'max_steer={math.degrees(self.max_steering_angle):.2f} deg, '
            f'timeout={self.command_timeout:.2f} s')

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _publish_stop(self) -> None:
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

    def _on_control_cmd(self, msg: VehicleState) -> None:
        self.last_command_time = self.get_clock().now()
        self.timeout_active = False

        if not math.isfinite(msg.velocity) or not math.isfinite(msg.steering):
            self.get_logger().error('Rejecting non-finite control command')
            self.brake_active = True
            self._publish_stop()
            return

        if abs(msg.gears) > 1e-6 and not self.warned_nonzero_gear:
            self.get_logger().warn(
                'gears is reserved and ignored; use velocity sign for direction')
            self.warned_nonzero_gear = True

        self.brake_active = msg.brake > self.brake_threshold
        if self.brake_active:
            self._publish_stop()
            return

        velocity = self._clamp(
            float(msg.velocity), -self.max_reverse_speed,
            self.max_forward_speed)
        steering_rad = math.radians(float(msg.steering))
        steering_rad = self._clamp(
            steering_rad, -self.max_steering_angle,
            self.max_steering_angle)

        # This is intentionally NOT the standard Twist yaw-rate formula.
        # cmd_vel_to_ackermann.py interprets angular.z as steering control input:
        # center steering angle = steer_gain * angular.z.
        command = Twist()
        command.linear.x = velocity
        command.angular.z = steering_rad / self.steer_gain
        self.cmd_pub.publish(command)

    def _on_odom(self, msg: Odometry) -> None:
        velocity = float(msg.twist.twist.linear.x)
        if math.isfinite(velocity):
            self.latest_velocity = velocity

    def _on_joint_states(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.joint_positions[name] = float(msg.position[index])

    def _center_steering_angle(self) -> float:
        left: Optional[float] = self.joint_positions.get(
            self.LEFT_STEER_JOINT)
        right: Optional[float] = self.joint_positions.get(
            self.RIGHT_STEER_JOINT)

        curvatures = []
        if left is not None:
            tangent = math.tan(left)
            denominator = self.wheelbase + 0.5 * self.track_width * tangent
            if abs(denominator) > 1e-9:
                curvatures.append(tangent / denominator)
        if right is not None:
            tangent = math.tan(right)
            denominator = self.wheelbase - 0.5 * self.track_width * tangent
            if abs(denominator) > 1e-9:
                curvatures.append(tangent / denominator)

        if not curvatures:
            return 0.0
        curvature = sum(curvatures) / len(curvatures)
        return math.atan(self.wheelbase * curvature)

    def _publish_feedback(self) -> None:
        feedback = VehicleState()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.header.frame_id = 'base_link'
        feedback.velocity = float(self.latest_velocity)
        feedback.steering = float(math.degrees(
            self._center_steering_angle()))
        feedback.brake = 1.0 if (self.brake_active or self.timeout_active) else 0.0
        feedback.gears = 0.0
        feedback.led = 'stop' if feedback.brake > 0.0 else 'auto'
        feedback.battary = float(self.simulated_battery)
        self.info_pub.publish(feedback)
        if self.state_pub is not None:
            self.state_pub.publish(feedback)

    def _command_watchdog(self) -> None:
        if self.last_command_time is None:
            return
        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if age <= self.command_timeout:
            return
        if not self.timeout_active:
            self.get_logger().warn(
                f'/control_cmd timeout ({age:.3f}s); forcing stop')
        self.timeout_active = True
        self.brake_active = True
        self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = C305SimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch may have already shut down the ROS context on SIGINT.  Only
        # publish the final stop while the publisher is still valid; the
        # runtime watchdog already keeps the vehicle stopped on lost input.
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
