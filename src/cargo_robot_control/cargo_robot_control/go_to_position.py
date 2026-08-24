import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class GoToPosition(Node):

    def __init__(self):
        super().__init__('go_to_position')

        # 실행할 때 바꿀 수 있는 값들
        self.declare_parameter('robot_namespace', 'robot1')
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('tolerance', 0.05)

        self.robot_namespace = self.get_parameter(
            'robot_namespace').value.strip('/')

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.tolerance = float(self.get_parameter('tolerance').value)

        odom_topic = (
            f'/{self.robot_namespace}/'
            'mecanum_drive_controller/odometry'
        )

        cmd_topic = (
            f'/{self.robot_namespace}/'
            'mecanum_drive_controller/reference'
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            cmd_topic,
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.current_odom = None
        self.arrived = False

        # 20 Hz로 속도 명령 전송
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'{self.robot_namespace} 이동 시작 → '
            f'목표 ({self.target_x:.2f}, {self.target_y:.2f})'
        )

    def odom_callback(self, msg):
        self.current_odom = msg

    def get_yaw(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.current_odom is None:
            return

        pose = self.current_odom.pose.pose

        current_x = pose.position.x
        current_y = pose.position.y

        dx = self.target_x - current_x
        dy = self.target_y - current_y

        distance = math.hypot(dx, dy)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if distance <= self.tolerance:
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.angular.z = 0.0

            self.publisher.publish(cmd)

            if not self.arrived:
                self.get_logger().info(
                    f'도착 완료! 현재 위치 '
                    f'({current_x:.2f}, {current_y:.2f})'
                )
                self.arrived = True

            return

        self.arrived = False

        # odom 좌표계의 목표 방향을
        # 로봇 기준(base_link) x/y 속도로 변환
        yaw = self.get_yaw(pose.orientation)

        local_x_error = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        local_y_error = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        vx = self.kp * local_x_error
        vy = self.kp * local_y_error

        # 너무 빨라지지 않도록 제한
        speed = math.hypot(vx, vy)

        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale

        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.angular.z = 0.0

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = GoToPosition()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    stop.header.frame_id = 'base_link'
    node.publisher.publish(stop)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
