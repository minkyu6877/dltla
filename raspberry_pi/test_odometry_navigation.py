#!/usr/bin/env python3

import math
import unittest

from odometry_navigation import (
    CoordinateMission,
    MecanumOdometry,
    Pose2D,
    Waypoint,
    follower_command,
    limit_turn_for_follower,
)


class MecanumOdometryTest(unittest.TestCase):
    def test_integrates_forward_wheel_rpm(self):
        odom = MecanumOdometry(0.03, 0.194, 0.30, Pose2D(0, 0, 0), False)
        fields = {"rpm": "60:60:60:60"}
        self.assertTrue(odom.update(fields, 10.0))
        self.assertTrue(odom.update(fields, 11.0))
        self.assertAlmostEqual(odom.pose.x, 2 * math.pi * 0.03, places=5)
        self.assertAlmostEqual(odom.pose.y, 0.0, places=5)

    def test_robot2_imu_sets_shared_heading(self):
        odom = MecanumOdometry(0.03, 0.194, 0.30, Pose2D(0, 0, 0), True)
        initial = {"rpm": "0:0:0:0", "imu_ok": "1", "att_deg": "0:0:10"}
        turned = {"rpm": "60:60:60:60", "imu_ok": "1", "att_deg": "0:0:100"}
        odom.update(initial, 1.0)
        odom.update(turned, 2.0)
        self.assertAlmostEqual(math.degrees(odom.pose.heading), 90.0, places=4)
        # Midpoint integration prevents a discontinuous 90-degree teleport.
        self.assertGreater(odom.pose.x, 0.0)
        self.assertGreater(odom.pose.y, 0.0)


class CoordinateMissionTest(unittest.TestCase):
    def mission(self, route):
        return CoordinateMission(route, 0.05, 4.0, 0.2, 0.04, 0.8, 1.6, 0.8)

    def test_starts_forward_toward_loading(self):
        mission = self.mission([Waypoint("LOADING", 2.2, 0.75)])
        mission.start()
        command = mission.guidance(Pose2D(0.99, 0.75, 0.0), 0.0)
        self.assertEqual(command.mode, "MOVE")
        self.assertGreater(command.vx_mps, 0.0)
        self.assertAlmostEqual(command.vy_mps, 0.0)

    def test_safe_return_is_reverse_without_turn(self):
        mission = self.mission([Waypoint("SAFE_REVERSE", 3.25, 2.20, reverse=True)])
        mission.start()
        heading = math.atan2(0.35, 0.30)
        command = mission.guidance(Pose2D(3.55, 2.55, heading), 0.0)
        self.assertEqual(command.mode, "MOVE")
        self.assertLess(command.vx_mps, 0.0)

    def test_follower_sweeps_wider_during_left_turn(self):
        command = follower_command(
            (0.0, 0.0, 0.2), "TURN", 30.0, 30.0, 0.012, 0.08, 24.7, 23.0
        )
        self.assertLess(command[1], 0.0)
        self.assertAlmostEqual(command[2], 0.2)

    def test_outer_follower_turn_respects_linear_speed_limit(self):
        leader = limit_turn_for_follower(
            (0.0, 0.0, 0.21), 30.0, 23.0, 24.7, 0.212
        )
        follower = follower_command(
            leader, "TURN", 30.0, 30.0, 0.012, 0.08, 24.7, 23.0
        )
        self.assertLessEqual(abs(follower[1]), 0.212 + 1e-9)

    def test_complete_simulation_route_visits_green_and_home(self):
        route = [
            Waypoint("LOADING", 2.20, 0.75),
            Waypoint("LOWER_RIGHT", 3.25, 0.75),
            Waypoint("UPPER_RIGHT", 3.25, 2.20),
            Waypoint("GREEN_DESTINATION", 3.55, 2.55, dwell_sec=2.0),
            Waypoint("SAFE_REVERSE", 3.25, 2.20, reverse=True),
            Waypoint("UPPER_LEFT", 0.75, 2.20),
            Waypoint("LOWER_LEFT", 0.75, 0.75),
            Waypoint("STANDBY", 0.99, 0.75),
        ]
        mission = self.mission(route)
        mission.start()
        pose = Pose2D(0.99, 0.75, 0.0)
        dt = 0.05
        green_dwell_seen = False
        for tick in range(4000):
            now = tick * dt
            guidance = mission.guidance(pose, now)
            if guidance.name == "GREEN_DESTINATION" and guidance.mode == "DWELL":
                green_dwell_seen = True
            if guidance.mode == "TURN":
                pose.heading += guidance.yaw_rate_rps * dt
            elif guidance.mode == "MOVE":
                cos_h, sin_h = math.cos(pose.heading), math.sin(pose.heading)
                pose.x += (
                    cos_h * guidance.vx_mps - sin_h * guidance.vy_mps
                ) * dt
                pose.y += (
                    sin_h * guidance.vx_mps + cos_h * guidance.vy_mps
                ) * dt
                pose.heading += guidance.yaw_rate_rps * dt
            if mission.completed:
                break
        self.assertTrue(mission.completed)
        self.assertTrue(green_dwell_seen)
        self.assertLess(math.dist((pose.x, pose.y), (0.99, 0.75)), 0.051)


if __name__ == "__main__":
    unittest.main()
