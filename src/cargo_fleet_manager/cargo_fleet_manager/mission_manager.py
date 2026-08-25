import copy
import heapq
import json
import math
import os
from collections import deque

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class MissionManager(Node):
    """Execute YAML waypoint missions using only UWB for X/Y feedback."""

    ROBOTS = ('robot1', 'robot2')
    # These rectangles match the shelves in warehouse_l_shape.sdf.  They are
    # enlarged by the robot clearance before planning, so commands never aim
    # through a rack even though the controller itself only follows points.
    # Test layout: empty floor. Add shelf rectangles here for obstacle tests.
    OBSTACLES = ()
    MAP_BOUNDS = (0.15, 3.85, 0.15, 2.85)
    GRID_SIZE = 0.10
    ROBOT_CLEARANCE = 0.28
    LOADING_POINT = (0.60, 0.50)
    STANDBY_POINT = (2.00, 0.40)

    def __init__(self):
        super().__init__('mission_manager')
        default_config = os.path.join(
            get_package_share_directory('cargo_fleet_manager'),
            'config', 'missions.yaml')
        self.declare_parameter('mission_config', default_config)
        self.config_path = str(self.get_parameter('mission_config').value)
        self.missions, control = self.load_config(self.config_path)

        self.tolerance = float(control.get('tolerance', 0.08))
        self.max_velocity = float(control.get('max_velocity', 0.30))
        self.uwb_timeout = float(control.get('uwb_timeout', 0.50))
        rate = float(control.get('control_rate_hz', 20.0))
        if min(self.tolerance, self.max_velocity, self.uwb_timeout, rate) <= 0:
            raise ValueError('Control values in missions.yaml must be positive')

        self.position = {robot: None for robot in self.ROBOTS}
        self.position_time = {robot: None for robot in self.ROBOTS}
        self.cmd_pub = {}
        for robot in self.ROBOTS:
            self.cmd_pub[robot] = self.create_publisher(
                TwistStamped,
                f'/{robot}/mecanum_drive_controller/reference', 10)
            self.create_subscription(
                PointStamped, f'/{robot}/uwb_position',
                lambda msg, r=robot: self.uwb_callback(msg, r), 10)

        self.create_subscription(
            String, '/fleet/mission', self.mission_callback, 10)
        self.state_pub = self.create_publisher(String, '/fleet/state', 10)

        self.state = 'STANDBY'
        self.mission_id = None
        self.route = []
        self.robot_count = 0
        self.assigned_robots = []
        self.waypoint_index = 0
        self.follower_delay = 0.0
        self.follower_offset = None
        self.command_history = deque()
        self.follower_command = self.make_command(0.0, 0.0)
        self.last_safety_reason = None
        self.delivery_request = None

        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f'Ready: {len(self.missions)} missions loaded from {self.config_path}')
        self.get_logger().info('State -> STANDBY; waiting on /fleet/mission')

    @staticmethod
    def load_config(path):
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        raw_missions = data.get('missions')
        if not isinstance(raw_missions, dict) or not raw_missions:
            raise ValueError(f'No missions configured in {path}')

        missions = {}
        for raw_id, raw in raw_missions.items():
            mission_id = str(raw_id).strip().upper()
            if not isinstance(raw, dict):
                raise ValueError(f'Mission {mission_id} must be a mapping')
            robots = int(raw.get('robots', 0))
            if robots not in (1, 2):
                raise ValueError(f'Mission {mission_id}: robots must be 1 or 2')
            raw_route = raw.get('route')
            if not isinstance(raw_route, list) or not raw_route:
                raise ValueError(f'Mission {mission_id}: route must not be empty')
            route = []
            for point in raw_route:
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError(
                        f'Mission {mission_id}: waypoint must be [x, y]')
                route.append((float(point[0]), float(point[1])))
            delay = float(raw.get('follower_delay', 0.0))
            if delay < 0:
                raise ValueError(
                    f'Mission {mission_id}: follower_delay cannot be negative')
            missions[mission_id] = {
                'robots': robots,
                'route': route,
                'follower_delay': delay,
            }
        return missions, data.get('control', {}) or {}

    def uwb_callback(self, msg, robot):
        self.position[robot] = (float(msg.point.x), float(msg.point.y))
        self.position_time[robot] = self.get_clock().now()

    def mission_callback(self, msg):
        raw_command = msg.data.strip()
        try:
            request = json.loads(raw_command)
        except json.JSONDecodeError:
            request = None
        if isinstance(request, dict) and (
                request.get('command') == 'DELIVERY' or
                ('dest_x' in request and 'dest_y' in request)):
            self.start_delivery(request)
            return

        mission_id = raw_command.upper()
        mission = self.missions.get(mission_id)
        if mission is None:
            self.stop_all()
            self.get_logger().error(
                f'Unknown mission ID {mission_id!r}; robots remain stopped')
            return
        if self.state == 'RUNNING':
            self.get_logger().warn(
                f'Rejected {mission_id}: {self.mission_id} is already running')
            return

        self.mission_id = mission_id
        self.route = list(mission['route'])
        self.robot_count = mission['robots']
        self.assigned_robots = list(self.ROBOTS[:self.robot_count])
        self.follower_delay = mission['follower_delay']
        self.waypoint_index = 0
        self.follower_offset = None
        self.command_history.clear()
        self.follower_command = self.make_command(0.0, 0.0)
        self.last_safety_reason = None
        self.state = 'RUNNING'
        self.stop_all()
        self.get_logger().info(
            f'Mission {mission_id} STARTED: robots={self.robot_count}, '
            f'waypoints={len(self.route)}, delay={self.follower_delay:.2f}s')
        self.log_waypoint()

    def start_delivery(self, request):
        """Start standby -> loading -> requested destination -> standby."""
        if self.state == 'RUNNING':
            self.get_logger().warn('Rejected delivery: another mission is running')
            return
        try:
            destination = (float(request['dest_x']), float(request['dest_y']))
        except (KeyError, TypeError, ValueError):
            self.get_logger().error('Delivery requires numeric dest_x and dest_y')
            return
        x_min, x_max, y_min, y_max = self.MAP_BOUNDS
        if not (x_min <= destination[0] <= x_max and
                y_min <= destination[1] <= y_max):
            self.get_logger().error('Destination is outside the warehouse map')
            return
        if self.is_blocked(*destination):
            self.get_logger().error('Destination is inside a shelf safety zone')
            return

        cargo_type = str(request.get('cargo_type', 'small')).lower()
        try:
            heavy = float(request.get('weight_kg', 0.0)) >= 5.0
        except (TypeError, ValueError):
            heavy = False
        self.robot_count = 2 if cargo_type in ('long', 'heavy', 'wide') or heavy else 1
        self.assigned_robots = list(self.ROBOTS[:self.robot_count])
        self.mission_id = 'DELIVERY'
        self.route = []
        self.delivery_request = {'destination': destination}
        self.waypoint_index = 0
        self.follower_offset = None
        self.follower_delay = float(request.get('follower_delay', 0.0))
        self.command_history.clear()
        self.follower_command = self.make_command(0.0, 0.0)
        self.last_safety_reason = None
        self.state = 'RUNNING'
        self.stop_all()
        self.get_logger().info(
            f'DELIVERY STARTED: robots={self.robot_count}, '
            f'loading={self.LOADING_POINT}, destination={destination}')

    def is_blocked(self, x, y):
        clearance = self.ROBOT_CLEARANCE
        for x_min, x_max, y_min, y_max in self.OBSTACLES:
            if x_min - clearance <= x <= x_max + clearance and \
                    y_min - clearance <= y <= y_max + clearance:
                return True
        return False

    def plan_segment(self, start, goal):
        """A* route through the warehouse grid, reduced to corner waypoints."""
        if not self.OBSTACLES:
            # The obstacle-free test layout should demonstrate direct travel
            # between the coloured zones rather than an artificial grid turn.
            return [goal]
        step = self.GRID_SIZE
        to_cell = lambda point: (round(point[0] / step), round(point[1] / step))
        to_point = lambda cell: (cell[0] * step, cell[1] * step)
        min_x, max_x, min_y, max_y = self.MAP_BOUNDS
        start_cell, goal_cell = to_cell(start), to_cell(goal)
        frontier = [(0.0, start_cell)]
        came_from = {start_cell: None}
        cost = {start_cell: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (current[0] + dx, current[1] + dy)
                point = to_point(nxt)
                if not (min_x <= point[0] <= max_x and min_y <= point[1] <= max_y) or \
                        self.is_blocked(*point):
                    continue
                new_cost = cost[current] + 1.0
                if new_cost >= cost.get(nxt, float('inf')):
                    continue
                cost[nxt] = new_cost
                priority = new_cost + abs(goal_cell[0] - nxt[0]) + abs(goal_cell[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current
        if goal_cell not in came_from:
            raise ValueError(f'No safe path to {goal}')
        cells = []
        current = goal_cell
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        corners = [to_point(cells[0])]
        previous_direction = None
        for previous, current in zip(cells, cells[1:]):
            direction = (current[0] - previous[0], current[1] - previous[1])
            if previous_direction is not None and direction != previous_direction:
                corners.append(to_point(previous))
            previous_direction = direction
        corners.append(goal)
        return corners[1:]

    def build_delivery_route(self):
        start = self.position['robot1']
        destination = self.delivery_request['destination']
        route = []
        for segment_start, segment_goal in (
                (start, self.LOADING_POINT),
                (self.LOADING_POINT, destination),
                (destination, self.STANDBY_POINT)):
            route.extend(self.plan_segment(segment_start, segment_goal))
        return route

    def make_command(self, vx, vy):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = float(vx)
        command.twist.linear.y = float(vy)
        return command

    def publish_command(self, robot, command):
        command.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub[robot].publish(command)

    def stop_robot(self, robot):
        self.publish_command(robot, self.make_command(0.0, 0.0))

    def stop_all(self):
        if hasattr(self, 'cmd_pub'):
            for robot in self.ROBOTS:
                self.stop_robot(robot)

    def positions_are_fresh(self):
        now = self.get_clock().now()
        for robot in self.assigned_robots:
            stamp = self.position_time[robot]
            if stamp is None:
                return False, f'waiting for {robot} UWB'
            age = (now - stamp).nanoseconds / 1e9
            if age > self.uwb_timeout:
                return False, f'{robot} UWB stale ({age:.2f}s)'
        return True, None

    def initialize_follower_offset(self):
        if self.robot_count != 2 or self.follower_offset is not None:
            return
        leader_x, leader_y = self.position['robot1']
        follower_x, follower_y = self.position['robot2']
        self.follower_offset = (
            follower_x - leader_x,
            follower_y - leader_y,
        )
        self.get_logger().info(
            'Robot 2 formation offset captured: '
            f'({self.follower_offset[0]:.2f}, {self.follower_offset[1]:.2f})')

    def goal_for(self, robot):
        x, y = self.route[self.waypoint_index]
        if robot == 'robot2' and self.follower_offset is not None:
            x += self.follower_offset[0]
            y += self.follower_offset[1]
        return x, y

    def at_goal(self, robot):
        goal_x, goal_y = self.goal_for(robot)
        x, y = self.position[robot]
        return math.hypot(goal_x - x, goal_y - y) <= self.tolerance

    def leader_command(self):
        if self.at_goal('robot1'):
            return self.make_command(0.0, 0.0)
        goal_x, goal_y = self.goal_for('robot1')
        x, y = self.position['robot1']
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        speed = min(self.max_velocity, distance)
        return self.make_command(speed * dx / distance, speed * dy / distance)

    def delayed_follower_command(self, leader_command):
        now_ns = self.get_clock().now().nanoseconds
        self.command_history.append((now_ns, copy.deepcopy(leader_command)))
        cutoff = now_ns - int(self.follower_delay * 1e9)
        while self.command_history and self.command_history[0][0] <= cutoff:
            _, self.follower_command = self.command_history.popleft()
        return copy.deepcopy(self.follower_command)

    def log_waypoint(self):
        x, y = self.route[self.waypoint_index]
        self.get_logger().info(
            f'{self.mission_id}: waypoint {self.waypoint_index + 1}/'
            f'{len(self.route)} -> ({x:.2f}, {y:.2f})')

    def control_loop(self):
        state = String()
        state.data = self.state
        self.state_pub.publish(state)
        if self.state != 'RUNNING':
            return

        fresh, reason = self.positions_are_fresh()
        if not fresh:
            self.stop_all()
            self.command_history.clear()
            self.follower_command = self.make_command(0.0, 0.0)
            if reason != self.last_safety_reason:
                self.get_logger().warn(f'SAFETY STOP: {reason}')
                self.last_safety_reason = reason
            return
        if self.last_safety_reason is not None:
            self.get_logger().info('Fresh UWB restored; mission resumed')
            self.last_safety_reason = None

        self.initialize_follower_offset()
        if self.delivery_request is not None and not self.route:
            try:
                self.route = self.build_delivery_route()
            except ValueError as error:
                self.get_logger().error(f'DELIVERY CANCELLED: {error}')
                self.stop_all()
                self.state = 'STANDBY'
                self.assigned_robots = []
                self.delivery_request = None
                return
            self.delivery_request = None
            self.log_waypoint()
        leader = self.leader_command()
        self.publish_command('robot1', leader)
        if self.robot_count == 2:
            # Real deployment boundary: replace this ROS publisher with the
            # future Raspberry Pi -> Robot 2 ESP32 transport adapter.
            self.publish_command(
                'robot2', self.delayed_follower_command(leader))
        else:
            self.stop_robot('robot2')

        # Only assigned robots are awaited. SMALL missions never wait for robot2.
        if not all(self.at_goal(robot) for robot in self.assigned_robots):
            return
        self.stop_all()
        self.command_history.clear()
        self.get_logger().info(
            f'{self.mission_id}: waypoint {self.waypoint_index + 1} reached')
        self.waypoint_index += 1
        if self.waypoint_index < len(self.route):
            self.follower_command = self.make_command(0.0, 0.0)
            self.log_waypoint()
            return

        finished_id = self.mission_id
        self.state = 'STANDBY'
        self.mission_id = None
        self.assigned_robots = []
        self.delivery_request = None
        self.get_logger().info(
            f'Mission {finished_id} COMPLETED; state -> STANDBY')


def main(args=None):
    rclpy.init(
        args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.stop_all()
    finally:
        if rclpy.ok():
            node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
