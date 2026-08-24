import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class FleetManager(Node):

    def __init__(self):
        super().__init__('fleet_manager')

        self.declare_parameter('cargo_type', 'long')
        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('robot_spacing', 1.4)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('tolerance', 0.05)

        self.cargo_type = str(
            self.get_parameter('cargo_type').value
        ).lower()

        self.target_x = float(
            self.get_parameter('target_x').value
        )

        self.target_y = float(
            self.get_parameter('target_y').value
        )

        self.robot_spacing = float(
            self.get_parameter('robot_spacing').value
        )

        self.max_speed = float(
            self.get_parameter('max_speed').value
        )

        self.kp = float(
            self.get_parameter('kp').value
        )

        self.tolerance = float(
            self.get_parameter('tolerance').value
        )

        self.odom = {
            'robot1': None,
            'robot2': None
        }

        self.arrived = {
            'robot1': False,
            'robot2': False
        }

        self.cmd_pub = {}

        for robot in ['robot1', 'robot2']:

            odom_topic = (
                f'/{robot}/'
                'mecanum_drive_controller/odometry'
            )

            cmd_topic = (
                f'/{robot}/'
                'mecanum_drive_controller/reference'
            )

            self.cmd_pub[robot] = self.create_publisher(
                TwistStamped,
                cmd_topic,
                10
            )

            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, r=robot: self.odom_callback(msg, r),
                10
            )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        if self.cargo_type == 'small':

            self.get_logger().info(
                'SMALL 화물 → Robot 1 한 대 배정'
            )

        elif self.cargo_type == 'long':

            self.get_logger().info(
                'LONG 화물 → Robot 1 + Robot 2 배정'
            )

        else:

            self.get_logger().error(
                f'알 수 없는 cargo_type: {self.cargo_type}'
            )

        self.get_logger().info(
            f'그룹 목적지 → '
            f'({self.target_x:.2f}, {self.target_y:.2f})'
        )

    def odom_callback(self, msg, robot):

        self.odom[robot] = msg

    def get_yaw(self, orientation):

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (
            w * z + x * y
        )

        cosy_cosp = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def stop_robot(self, robot):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.header.frame_id = 'base_link'

        self.cmd_pub[robot].publish(cmd)

    def move_robot(
        self,
        robot,
        goal_x,
        goal_y
    ):

        odom = self.odom[robot]

        if odom is None:
            return

        pose = odom.pose.pose

        current_x = pose.position.x
        current_y = pose.position.y

        dx = goal_x - current_x
        dy = goal_y - current_y

        distance = math.hypot(dx, dy)

        if distance <= self.tolerance:

            self.stop_robot(robot)

            if not self.arrived[robot]:

                self.get_logger().info(
                    f'{robot} 도착 → '
                    f'({current_x:.2f}, '
                    f'{current_y:.2f})'
                )

                self.arrived[robot] = True

            return

        if self.arrived[robot]:
            self.stop_robot(robot)
            return

        yaw = self.get_yaw(
            pose.orientation
        )

        local_x = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        local_y = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        vx = self.kp * local_x
        vy = self.kp * local_y

        speed = math.hypot(vx, vy)

        if speed > self.max_speed:

            scale = (
                self.max_speed / speed
            )

            vx *= scale
            vy *= scale

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.angular.z = 0.0

        self.cmd_pub[robot].publish(cmd)

    def control_loop(self):

        if self.cargo_type == 'small':

            # 소형 화물: Robot 1만 사용
            self.move_robot(
                'robot1',
                self.target_x,
                self.target_y
            )

            self.stop_robot('robot2')

        elif self.cargo_type == 'long':

            # 긴 화물:
            # 그룹 중심 기준 위/아래로 배치
            half_spacing = (
                self.robot_spacing / 2.0
            )

            robot1_y = (
                self.target_y +
                half_spacing
            )

            robot2_y = (
                self.target_y -
                half_spacing
            )

            self.move_robot(
                'robot1',
                self.target_x,
                robot1_y
            )

            self.move_robot(
                'robot2',
                self.target_x,
                robot2_y
            )


def main(args=None):

    rclpy.init(args=args)

    node = FleetManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.stop_robot('robot1')
        node.stop_robot('robot2')

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
