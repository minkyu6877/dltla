import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class UwbSimulator(Node):
    # Gazebo controller odometry starts from (0, 0) for each robot.  UWB must
    # instead report the shared warehouse coordinate frame used by missions.
    WORLD_OFFSETS = {
        'robot1': (1.74, 0.40),
        'robot2': (2.27, 0.40),
    }

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
        self.gap_pub = self.create_publisher(
            Float32,
            '/robot2/ultrasonic_gap',
            10
        )
        self.positions = {}

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

        offset_x, offset_y = self.WORLD_OFFSETS[robot_name]
        msg.point.x = offset_x + odom.pose.pose.position.x
        msg.point.y = offset_y + odom.pose.pose.position.y
        msg.point.z = 0.0

        pub.publish(msg)
        self.positions[robot_name] = (msg.point.x, msg.point.y)
        if len(self.positions) == 2:
            # The actual sensor reads the clear gap between inward-facing
            # robot bodies, not the centre-to-centre distance.
            center_distance = ((
                self.positions['robot1'][0] - self.positions['robot2'][0]) ** 2 + (
                self.positions['robot1'][1] - self.positions['robot2'][1]) ** 2
            ) ** 0.5
            self.gap_pub.publish(Float32(data=max(0.0, center_distance - 0.23)))

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
