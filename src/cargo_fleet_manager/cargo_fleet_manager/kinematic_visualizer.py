"""Deterministic, no-physics Gazebo route visualization for the two robots."""

import math

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Float32


class KinematicVisualizer(Node):
    """Move static Gazebo models from mission velocity requests.

    This is a route-demonstration simulator, not a wheel-contact simulator.
    Hardware motion remains the IMU/encoder/ultrasonic implementation.
    """

    ROBOTS = ('robot1', 'robot2')
    HOME_POSITIONS = {'robot1': (0.99, 0.40), 'robot2': (0.46, 0.40)}
    MAP_X = (0.12, 3.88)
    MAP_Y = (0.12, 2.88)
    BODY_LENGTH = 0.23

    def __init__(self):
        super().__init__('kinematic_visualizer')
        self.positions = {
            robot: list(position) for robot, position in self.HOME_POSITIONS.items()
        }
        self.commands = {robot: TwistStamped() for robot in self.ROBOTS}
        self.pending = {robot: None for robot in self.ROBOTS}
        self.pose_clients = {
            robot: self.create_client(
                SetEntityPose, '/world/warehouse_l_shape/set_pose')
            for robot in self.ROBOTS
        }
        self.position_publishers = {
            robot: self.create_publisher(PointStamped, f'/{robot}/uwb_position', 10)
            for robot in self.ROBOTS
        }
        self.gap_publisher = self.create_publisher(
            Float32, '/robot2/ultrasonic_gap', 10)
        for robot in self.ROBOTS:
            self.create_subscription(
                TwistStamped,
                f'/{robot}/mecanum_drive_controller/reference',
                lambda message, name=robot: self.command_callback(message, name),
                10,
            )
        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.step)
        self.get_logger().info('Kinematic route visualizer started (simulation only)')

    def command_callback(self, message, robot):
        self.commands[robot] = message

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            return
        for robot in self.ROBOTS:
            command = self.commands[robot].twist.linear
            position = self.positions[robot]
            position[0] = min(self.MAP_X[1], max(
                self.MAP_X[0], position[0] + command.x * dt))
            position[1] = min(self.MAP_Y[1], max(
                self.MAP_Y[0], position[1] + command.y * dt))
            self.publish_position(robot, now)
            self.update_model_pose(robot)
        self.publish_gap()

    def publish_position(self, robot, now):
        message = PointStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'world'
        message.point.x, message.point.y = self.positions[robot]
        self.position_publishers[robot].publish(message)

    def publish_gap(self):
        first, second = self.positions['robot1'], self.positions['robot2']
        center_distance = math.dist(first, second)
        self.gap_publisher.publish(Float32(
            data=max(0.0, center_distance - self.BODY_LENGTH)))

    def update_model_pose(self, robot):
        pending = self.pending[robot]
        if pending is not None and not pending.done():
            return
        client = self.pose_clients[robot]
        if not client.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = f'cargo_robot_{robot[-1]}'
        request.entity.type = Entity.MODEL
        request.pose.position.x, request.pose.position.y = self.positions[robot]
        request.pose.position.z = 0.0
        request.pose.orientation.w = 1.0
        self.pending[robot] = client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
