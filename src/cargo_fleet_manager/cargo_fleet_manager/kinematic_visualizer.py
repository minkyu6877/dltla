"""Deterministic, no-physics Gazebo route visualization for the two robots."""

import math

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Float32, String


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
    TARGET_CENTER_DISTANCE = 0.53
    PATH_SAMPLE_DISTANCE = 0.005
    TARGET_SNAP_DISTANCE = 0.02

    def __init__(self):
        super().__init__('kinematic_visualizer')
        self.positions = {
            robot: list(position) for robot, position in self.HOME_POSITIONS.items()
        }
        self.headings = {robot: 0.0 for robot in self.ROBOTS}
        self.commands = {robot: TwistStamped() for robot in self.ROBOTS}
        self.leader_path = [
            tuple(self.positions['robot2']),
            tuple(self.positions['robot1']),
        ]
        self.following = False
        self.stationary_time = 0.0
        self.current_target = None
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
        self.create_subscription(
            PointStamped, '/fleet/current_target', self.target_callback, 10)
        self.create_subscription(
            String, '/fleet/state', self.state_callback, 10)
        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.step)
        self.get_logger().info('Kinematic route visualizer started (simulation only)')

    def command_callback(self, message, robot):
        self.commands[robot] = message

    def target_callback(self, message):
        self.current_target = (message.point.x, message.point.y)

    def state_callback(self, message):
        if message.data != 'STANDBY':
            return
        self.positions = {
            robot: list(position) for robot, position in self.HOME_POSITIONS.items()
        }
        self.headings = {robot: 0.0 for robot in self.ROBOTS}
        self.commands = {robot: TwistStamped() for robot in self.ROBOTS}
        self.leader_path = [
            tuple(self.positions['robot2']),
            tuple(self.positions['robot1']),
        ]
        self.following = False
        self.stationary_time = 0.0

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            return
        leader_command = self.commands['robot1'].twist.linear
        follower_command = self.commands['robot2'].twist.linear
        leader_moving = math.hypot(leader_command.x, leader_command.y) > 0.001
        self.following = math.hypot(
            follower_command.x, follower_command.y) > 0.001

        self.advance_leader(leader_command, dt)
        if leader_moving and math.dist(
                self.leader_path[-1], self.positions['robot1']) >= self.PATH_SAMPLE_DISTANCE:
            self.leader_path.append(tuple(self.positions['robot1']))

        if self.following:
            previous = tuple(self.positions['robot2'])
            target = self.follower_target()
            self.positions['robot2'][:] = target
            dx, dy = target[0] - previous[0], target[1] - previous[1]
            if math.hypot(dx, dy) > 0.0001:
                self.headings['robot2'] = math.atan2(dy, dx)
            self.stationary_time = 0.0
        elif not leader_moving:
            self.stationary_time += dt
            if self.stationary_time >= 0.5:
                self.leader_path = [
                    tuple(self.positions['robot2']),
                    tuple(self.positions['robot1']),
                ]

        for robot in self.ROBOTS:
            self.publish_position(robot, now)
            self.update_model_pose(robot)
        self.publish_gap()

    def advance_leader(self, command, dt):
        position = self.positions['robot1']
        position[0] = min(self.MAP_X[1], max(
            self.MAP_X[0], position[0] + command.x * dt))
        position[1] = min(self.MAP_Y[1], max(
            self.MAP_Y[0], position[1] + command.y * dt))
        if self.current_target is not None and math.dist(
                position, self.current_target) <= self.TARGET_SNAP_DISTANCE:
            position[:] = self.current_target
        if math.hypot(command.x, command.y) > 0.001:
            self.headings['robot1'] = math.atan2(command.y, command.x)

    def follower_target(self):
        """Return the point 0.53 m behind Robot 1 along its travelled path."""
        remaining = self.TARGET_CENTER_DISTANCE
        current = tuple(self.positions['robot1'])
        for previous in reversed(self.leader_path):
            segment = math.dist(current, previous)
            if segment >= remaining and segment > 0.0:
                ratio = remaining / segment
                return [
                    current[0] + (previous[0] - current[0]) * ratio,
                    current[1] + (previous[1] - current[1]) * ratio,
                ]
            remaining -= segment
            current = previous
        return list(self.leader_path[0])

    def publish_position(self, robot, now):
        message = PointStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'world'
        message.point.x, message.point.y = self.positions[robot]
        self.position_publishers[robot].publish(message)

    def publish_gap(self):
        if self.following:
            self.gap_publisher.publish(Float32(
                data=self.TARGET_CENTER_DISTANCE - self.BODY_LENGTH))
            return
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
        request.pose.orientation.z = math.sin(self.headings[robot] / 2.0)
        request.pose.orientation.w = math.cos(self.headings[robot] / 2.0)
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
