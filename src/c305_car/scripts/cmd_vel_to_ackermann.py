#!/usr/bin/env python3
"""把 /cmd_vel 转换成阿克曼转向指令。

订阅:
  /cmd_vel                     geometry_msgs/Twist

发布:
  /steering_controller/commands  std_msgs/Float64MultiArray
                                 顺序 [左前, 右前]，单位 rad
  /ackermann_cmd_vel             geometry_msgs/Twist
                                 交给 diff_drive_controller 驱动后轮

映射关系:
  前轮转角 delta = clamp(steer_gain * angular.z, -max, +max)
  后轮角速度  omega = linear.x * tan(delta) / wheelbase

这样任何发布 /cmd_vel 的键盘遥控节点（teleop_twist_keyboard 等）
都会带动前轮转向；只按左右键（linear.x = 0）时前轮也会原地打方向。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermann(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        # 与 urdf/c305_car.urdf.xacro 保持一致；修改 L / W 后需同步这里。
        self.wheelbase = self.declare_parameter('wheelbase', 1.152).value
        self.track_width = self.declare_parameter('track_width', 0.902).value
        self.max_steering_angle = self.declare_parameter(
            'max_steering_angle', 0.5).value
        # angular.z = 1.0 时对应的前轮转角（rad）
        self.steer_gain = self.declare_parameter('steer_gain', 0.5).value
        # 是否按内外轮差角（阿克曼）分别计算左右前轮
        self.use_ackermann = self.declare_parameter('ackermann', True).value

        self.steer_pub = self.create_publisher(
            Float64MultiArray, 'steering_controller/commands', 10)
        self.drive_pub = self.create_publisher(
            Twist, 'ackermann_cmd_vel', 10)
        self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info(
            'cmd_vel -> Ackermann 桥已启动: '
            f'wheelbase={self.wheelbase:.3f} m, '
            f'track={self.track_width:.3f} m, '
            f'max_steer={self.max_steering_angle:.2f} rad, '
            f'ackermann={self.use_ackermann}')

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def on_cmd_vel(self, msg):
        velocity = msg.linear.x
        delta = self._clamp(
            self.steer_gain * msg.angular.z, self.max_steering_angle)

        if self.use_ackermann and abs(delta) > 1e-4:
            # 以车体后轴中心为参考的转弯半径（带符号，左转为正）
            radius = self.wheelbase / math.tan(delta)
            delta_left = math.atan(
                self.wheelbase / (radius - self.track_width / 2.0))
            delta_right = math.atan(
                self.wheelbase / (radius + self.track_width / 2.0))
        else:
            delta_left = delta_right = delta

        steer_msg = Float64MultiArray()
        steer_msg.data = [
            self._clamp(delta_left, self.max_steering_angle),
            self._clamp(delta_right, self.max_steering_angle),
        ]
        self.steer_pub.publish(steer_msg)

        # 用前轮转角反算后轴需要的角速度，保证后轮差速与转向几何一致
        drive_msg = Twist()
        drive_msg.linear.x = velocity
        drive_msg.angular.z = velocity * math.tan(delta) / self.wheelbase
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
