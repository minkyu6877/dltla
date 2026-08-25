"""Keep Gazebo's visual robots on the planned planar warehouse path.

The real robots use their wheel encoders / IMU for motion.  Gazebo's simple
wheel geometry deliberately does not attempt to model individual mecanum
rollers, so this simulation-only node mirrors the controller odometry into
the Gazebo model pose.  It makes the route demonstration stable without
claiming to be a tyre-contact validation.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class KinematicVisualizer(Node):
    """Apply the warehouse-frame simulated UWB position to each model."""

    ROBOTS = ('robot1', 'robot2')
    MIN_UPDATE_DISTANCE = 0.015

    def __init__(self):
        super().__init__('kinematic_visualizer')
        self.positions = {robot: None for robot in self.ROBOTS}
        self.last_sent = {robot: None for robot in self.ROBOTS}
        self.pending = {robot: None for robot in self.ROBOTS}
        self.clients = {
            robot: self.create_client(
                SetEntityPose, '/world/warehouse_l_shape/set_pose')
            for robot in self.ROBOTS
        }
        for robot in self.ROBOTS:
            self.create_subscription(
                PointStamped,
                f'/{robot}/uwb_position',
                lambda message, name=robot: self.position_callback(message, name),
                10,
            )
        self.create_timer(0.10, self.update_models)
        self.get_logger().info('Kinematic visualizer started (simulation only)')

    def position_callback(self, message, robot):
        self.positions[robot] = (message.point.x, message.point.y)

    def update_models(self):
        for robot in self.ROBOTS:
            position = self.positions[robot]
            request_in_flight = self.pending[robot]
            if position is None or (
                    request_in_flight is not None and
                    not request_in_flight.done()):
                continue
            previous = self.last_sent[robot]
            if previous is not None and math.dist(position, previous) < self.MIN_UPDATE_DISTANCE:
                continue
            client = self.clients[robot]
            if not client.service_is_ready():
                continue
            request = SetEntityPose.Request()
            request.entity.name = f'cargo_robot_{robot[-1]}'
            request.entity.type = Entity.MODEL
            request.pose.position.x = float(position[0])
            request.pose.position.y = float(position[1])
            request.pose.position.z = 0.0
            request.pose.orientation.w = 1.0
            self.pending[robot] = client.call_async(request)
            self.last_sent[robot] = position


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
