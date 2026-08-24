import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


class UwbSimulator(Node):

    def __init__(self):
        super().__init__('uwb_simulator')

        self.pub1 = self.create_publisher(
            PointStamped,
            '/robot1/uwb_position',
            10
        )

        self.pub2 = self.create_publisher(
            PointStamped,
            '/robot2/uwb_position',
            10
        )

        self.create_subscription(
            Odometry,
            '/robot1/mecanum_drive_controller/odometry',
            self.robot1_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/robot2/mecanum_drive_controller/odometry',
            self.robot2_callback,
            10
        )

        self.get_logger().info(
            'UWB Simulator 시작'
        )

    def publish_position(self, odom, pub, robot_name):

        msg = PointStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.point.x = odom.pose.pose.position.x
        msg.point.y = odom.pose.pose.position.y
        msg.point.z = 0.0

        pub.publish(msg)

    def robot1_callback(self, msg):
        self.publish_position(
            msg,
            self.pub1,
            'robot1'
        )

    def robot2_callback(self, msg):
        self.publish_position(
            msg,
            self.pub2,
            'robot2'
        )


def main(args=None):

    rclpy.init(args=args)

    node = UwbSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
