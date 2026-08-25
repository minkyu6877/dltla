import heapq
import json
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32, String


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
    # The body plus wheels occupy about 0.35 m across.  Keep an additional
    # margin so the two simulated robots never enter contact distance.
    MIN_ROBOT_SEPARATION = 0.44
    LOADING_POINT = (2.20, 0.75)
    DEFAULT_HOME_POSITIONS = {
        'robot1': (0.99, 0.75),
        'robot2': (0.46, 0.75),
    }

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
        self.home_positions = self.load_home_positions(
            control.get('home_positions'))
        self.standby_heading = math.radians(
            float(control.get('standby_heading_deg', 180.0)))
        self.robot2_uses_uwb = bool(control.get('robot2_uses_uwb', False))
        self.robot2_target_gap = float(control.get('robot2_target_gap_m', 0.30))
        self.robot2_gap_tolerance = float(
            control.get('robot2_gap_tolerance_m', 0.05))
        self.robot2_emergency_gap = float(
            control.get('robot2_emergency_gap_m', 0.20))
        self.robot2_gap_timeout = float(control.get('robot2_gap_timeout', 0.30))
        self.robot2_gap_kp = float(control.get('robot2_gap_kp', 0.80))
        if min(self.robot2_target_gap, self.robot2_gap_tolerance,
               self.robot2_emergency_gap, self.robot2_gap_timeout) <= 0 or \
                self.robot2_emergency_gap >= self.robot2_target_gap:
            raise ValueError('Robot 2 ultrasonic gap settings are invalid')

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

        self.robot2_gap = None
        self.robot2_gap_time = None
        self.create_subscription(
            Float32, '/robot2/ultrasonic_gap', self.ultrasonic_callback, 10)

        self.create_subscription(
            String, '/fleet/mission', self.mission_callback, 10)
        self.state_pub = self.create_publisher(String, '/fleet/state', 10)
        self.target_pub = self.create_publisher(
            PointStamped, '/fleet/current_target', 10)

        self.state = 'HOMING'
        self.mission_id = None
        self.route = []
        self.robot_count = 0
        self.assigned_robots = []
        self.waypoint_index = 0
        self.last_safety_reason = None
        self.last_proximity_reason = None
        self.delivery_request = None
        self.home_routes = {}
        self.home_waypoint_index = {}
        self.homing_planned = False
        self.homing_robots = ()

        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f'Ready: {len(self.missions)} missions loaded from {self.config_path}')
        self.start_homing('startup')

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

    @classmethod
    def load_home_positions(cls, raw_positions):
        raw_positions = raw_positions or cls.DEFAULT_HOME_POSITIONS
        homes = {}
        for robot in cls.ROBOTS:
            point = raw_positions.get(robot) if isinstance(raw_positions, dict) else None
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError(
                    f'home_positions.{robot} must be [x, y] in missions.yaml')
            homes[robot] = (float(point[0]), float(point[1]))
        return homes

    @staticmethod
    def load_point(raw_point, name):
        if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 2:
            raise ValueError(f'{name} must be [x, y] in missions.yaml')
        return float(raw_point[0]), float(raw_point[1])

    def uwb_callback(self, msg, robot):
        self.position[robot] = (float(msg.point.x), float(msg.point.y))
        self.position_time[robot] = self.get_clock().now()

    def ultrasonic_callback(self, msg):
        self.robot2_gap = float(msg.data)
        self.robot2_gap_time = self.get_clock().now()

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
        if self.state != 'STANDBY':
            self.get_logger().warn(
                f'Rejected {mission_id}: fleet is {self.state.lower()}')
            return

        self.mission_id = mission_id
        self.route = list(mission['route'])
        self.robot_count = mission['robots']
        self.assigned_robots = list(self.ROBOTS[:self.robot_count])
        self.waypoint_index = 0
        self.last_safety_reason = None
        self.last_proximity_reason = None
        self.state = 'RUNNING'
        self.stop_all()
        self.get_logger().info(
            f'Mission {mission_id} STARTED: robots={self.robot_count}, '
            f'waypoints={len(self.route)}')
        self.log_waypoint()

    def start_delivery(self, request):
        """Start standby -> loading -> requested destination -> standby."""
        if self.state != 'STANDBY':
            self.get_logger().warn(
                f'Rejected delivery: fleet is {self.state.lower()}')
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
        self.last_safety_reason = None
        self.last_proximity_reason = None
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
                (destination, self.home_positions['robot1'])):
            route.extend(self.plan_segment(segment_start, segment_goal))
        return route

    def make_command(self, vx, vy):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = float(vx)
        command.twist.linear.y = float(vy)
        return command

    def make_world_command(self, world_vx, world_vy):
        """Build Gazebo's map/odometry-frame mecanum velocity command."""
        # The Gazebo reference topic already applies the controller's frame
        # convention.  A second yaw rotation here reverses navigation.
        return self.make_command(world_vx, world_vy)

    def publish_command(self, robot, command):
        command.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub[robot].publish(command)

    def stop_robot(self, robot):
        self.publish_command(robot, self.make_command(0.0, 0.0))

    def stop_all(self):
        if hasattr(self, 'cmd_pub'):
            for robot in self.ROBOTS:
                self.stop_robot(robot)

    def robots_too_close(self):
        """Stop before the two robot footprints can overlap."""
        robot1_position = self.position['robot1']
        robot2_position = self.position['robot2']
        if robot1_position is None or robot2_position is None:
            return False
        distance = math.dist(robot1_position, robot2_position)
        if distance >= self.MIN_ROBOT_SEPARATION:
            if self.last_proximity_reason is not None:
                self.get_logger().info('Robot separation restored; motion resumed')
                self.last_proximity_reason = None
            return False
        self.stop_all()
        reason = (
            f'ROBOT PROXIMITY STOP: separation={distance:.2f} m '
            f'(minimum={self.MIN_ROBOT_SEPARATION:.2f} m)')
        if reason != self.last_proximity_reason:
            self.get_logger().warn(reason)
            self.last_proximity_reason = reason
        return True

    def positions_are_fresh(self):
        return self.positions_are_fresh_for(self.assigned_robots)

    def positions_are_fresh_for(self, robots):
        now = self.get_clock().now()
        for robot in robots:
            if robot == 'robot2' and not self.robot2_uses_uwb:
                continue
            stamp = self.position_time[robot]
            if stamp is None:
                return False, f'waiting for {robot} UWB'
            age = (now - stamp).nanoseconds / 1e9
            if age > self.uwb_timeout:
                return False, f'{robot} UWB stale ({age:.2f}s)'
        return True, None

    def has_position_feedback(self, robot):
        return robot != 'robot2' or self.robot2_uses_uwb

    def start_homing(self, reason):
        """Send each robot to its own standby slot after UWB is available."""
        self.state = 'HOMING'
        self.mission_id = None
        self.route = []
        self.robot_count = 0
        self.assigned_robots = []
        self.waypoint_index = 0
        self.last_safety_reason = None
        self.last_proximity_reason = None
        self.delivery_request = None
        self.home_routes = {}
        self.home_waypoint_index = {robot: 0 for robot in self.ROBOTS}
        self.homing_planned = False
        self.homing_robots = tuple(
            robot for robot in self.ROBOTS if self.has_position_feedback(robot))
        self.stop_all()
        self.get_logger().info(
            f'HOMING STARTED ({reason}); waiting for UWB before moving to '
            f'robot1={self.home_positions["robot1"]}')
        if not self.robot2_uses_uwb:
            self.get_logger().info(
                f'Robot 2 has no UWB: hold it at manual standby slot '
                f'{self.home_positions["robot2"]}')

    def at_point(self, robot, goal):
        x, y = self.position[robot]
        return math.hypot(goal[0] - x, goal[1] - y) <= self.tolerance

    def command_to_point(self, robot, goal):
        if self.at_point(robot, goal):
            return self.make_command(0.0, 0.0)
        x, y = self.position[robot]
        dx, dy = goal[0] - x, goal[1] - y
        distance = math.hypot(dx, dy)
        speed = min(self.max_velocity, distance)
        return self.make_world_command(speed * dx / distance, speed * dy / distance)

    def control_homing(self):
        fresh, reason = self.positions_are_fresh_for(self.homing_robots)
        if not fresh:
            self.stop_all()
            if reason != self.last_safety_reason:
                self.get_logger().warn(f'HOMING SAFETY STOP: {reason}')
                self.last_safety_reason = reason
            return
        if self.last_safety_reason is not None:
            self.get_logger().info('Fresh UWB restored; homing resumed')
            self.last_safety_reason = None
        if self.robots_too_close():
            return

        if not self.homing_planned:
            try:
                self.home_routes = {
                    robot: self.plan_segment(
                        self.position[robot], self.home_positions[robot])
                    for robot in self.homing_robots
                }
            except ValueError as error:
                self.stop_all()
                self.state = 'HOMING_BLOCKED'
                self.get_logger().error(f'HOMING BLOCKED: {error}')
                return
            self.homing_planned = True
            self.get_logger().info('HOMING route ready; moving to standby slots')

        all_home = True
        for robot in self.homing_robots:
            route = self.home_routes[robot]
            index = self.home_waypoint_index[robot]
            if index >= len(route):
                self.stop_robot(robot)
                continue
            goal = route[index]
            if self.at_point(robot, goal):
                self.home_waypoint_index[robot] += 1
                self.stop_robot(robot)
                if self.home_waypoint_index[robot] < len(route):
                    all_home = False
                continue
            all_home = False
            self.publish_command(robot, self.command_to_point(robot, goal))

        if not all_home:
            return
        self.stop_all()
        self.state = 'STANDBY'
        self.get_logger().info(
            'HOMING COMPLETE; state -> STANDBY; waiting on /fleet/mission')

    def goal_for(self, robot):
        return self.route[self.waypoint_index]

    def at_goal(self, robot):
        goal_x, goal_y = self.goal_for(robot)
        x, y = self.position[robot]
        return math.hypot(goal_x - x, goal_y - y) <= self.tolerance

    def assigned_robots_at_goal(self):
        return all(
            self.at_goal(robot)
            for robot in self.assigned_robots
            if self.has_position_feedback(robot))

    def ultrasonic_gap_is_fresh(self):
        if self.robot2_gap is None or self.robot2_gap_time is None:
            return False, 'waiting for robot2 ultrasonic gap'
        age = (self.get_clock().now() - self.robot2_gap_time).nanoseconds / 1e9
        if age > self.robot2_gap_timeout:
            return False, f'robot2 ultrasonic gap stale ({age:.2f}s)'
        if self.robot2_gap < 0.0:
            return False, 'robot2 ultrasonic gap is invalid'
        return True, None

    def ultrasonic_follower_command(self, leader_command):
        """Gap-controlled request for Robot 2's IMU/encoder drive controller."""
        gap = self.robot2_gap
        if gap <= self.robot2_emergency_gap:
            return self.make_command(0.0, 0.0)
        error = gap - self.robot2_target_gap
        # Robot 2 follows behind Robot 1 in the +X travel direction.  When
        # the gap becomes too small it slows; when it opens it catches up.
        scale = 1.0 + self.robot2_gap_kp * error
        if abs(error) <= self.robot2_gap_tolerance:
            scale = 1.0
        scale = max(0.25, min(1.25, scale))
        return self.make_command(
            leader_command.twist.linear.x * scale,
            leader_command.twist.linear.y * scale)

    def leader_command(self):
        if self.at_goal('robot1'):
            return self.make_command(0.0, 0.0)
        goal_x, goal_y = self.goal_for('robot1')
        x, y = self.position['robot1']
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        speed = min(self.max_velocity, distance)
        return self.make_world_command(speed * dx / distance, speed * dy / distance)

    def log_waypoint(self):
        x, y = self.route[self.waypoint_index]
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'world'
        target.point.x = x
        target.point.y = y
        self.target_pub.publish(target)
        self.get_logger().info(
            f'{self.mission_id}: waypoint {self.waypoint_index + 1}/'
            f'{len(self.route)} -> ({x:.2f}, {y:.2f})')

    def control_loop(self):
        state = String()
        state.data = self.state
        self.state_pub.publish(state)
        if self.state == 'HOMING':
            self.control_homing()
            return
        if self.state != 'RUNNING':
            return

        fresh, reason = self.positions_are_fresh()
        if not fresh:
            self.stop_all()
            if reason != self.last_safety_reason:
                self.get_logger().warn(f'SAFETY STOP: {reason}')
                self.last_safety_reason = reason
            return
        if self.robot_count == 2:
            gap_fresh, gap_reason = self.ultrasonic_gap_is_fresh()
            if not gap_fresh:
                self.stop_all()
                if gap_reason != self.last_safety_reason:
                    self.get_logger().warn(f'SAFETY STOP: {gap_reason}')
                    self.last_safety_reason = gap_reason
                return
            if self.robot2_gap <= self.robot2_emergency_gap:
                self.stop_all()
                reason = (
                    f'ROBOT 2 ULTRASONIC STOP: gap={self.robot2_gap:.2f} m '
                    f'(minimum={self.robot2_emergency_gap:.2f} m)')
                if reason != self.last_safety_reason:
                    self.get_logger().warn(reason)
                    self.last_safety_reason = reason
                return
        if self.last_safety_reason is not None:
            self.get_logger().info('Fresh UWB restored; mission resumed')
            self.last_safety_reason = None
        if self.robots_too_close():
            return

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
            # Robot 2 firmware uses IMU/encoder feedback to execute this
            # request; the ultrasonic gap only adjusts its pace.
            self.publish_command(
                'robot2', self.ultrasonic_follower_command(leader))
        else:
            self.stop_robot('robot2')

        # Only assigned robots are awaited. SMALL missions never wait for robot2.
        if not self.assigned_robots_at_goal():
            return
        self.get_logger().info(
            f'{self.mission_id}: waypoint {self.waypoint_index + 1} reached')
        self.stop_all()
        self.waypoint_index += 1
        if self.waypoint_index < len(self.route):
            self.log_waypoint()
            return

        self.get_logger().info(
            f'Mission {self.mission_id} COMPLETED; returning to standby slots')
        self.start_homing('delivery complete')


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
