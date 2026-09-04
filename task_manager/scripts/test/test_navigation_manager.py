#!/usr/bin/env python3

"""
Navigation subtask tests.

A few basic checks on the base, the lidar and nav_central run first, then the
NavigationTasks functions.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray

from frida_constants.navigation_constants import SCAN_TOPIC
from task_manager.utils.status import Status
from task_manager.utils.task import Task
from task_manager.utils.logger import Logger
from task_manager.subtask_managers.nav_tasks import NavigationTasks

AXIS_ERRORS_TOPIC = "/odrive/axis_errors"
AXIS_STATES_TOPIC = "/odrive/axis_states"
BUS_VOLTAGE_TOPIC = "/odrive/bus_voltage"
WHEEL_VEL_TOPIC = "/odrive/vel_est"

NUM_WHEELS = 4
AXIS_STATE_CLOSED_LOOP = 8
MIN_BUS_VOLTAGE = 20.0
TOPIC_TIMEOUT = 5.0

WHEEL_TEST_WZ = 0.6
WHEEL_TEST_TIME = 1.5
WHEEL_MIN_VEL = 0.1

EXPLORE_STEP = 0.4


def _front_point(x: float = 1.2) -> PointStamped:
    """A point `x` meters straight ahead of the robot (base_link frame)."""
    p = PointStamped()
    p.header.frame_id = "base_link"
    p.point.x = x
    return p


# TUPLA DE REGRESO DE IDA ARGS
class TestNavigationManager(Node):
    def __init__(self):
        super().__init__("NavigationTaskManager")
        self.logs = self.declare_parameter("clear_logs", True).value
        self.mocked = self.declare_parameter("mocked", False).value
        self.task_to_test = Task[self.declare_parameter("task", Task.DEBUG.name).value]
        self.run_basics = self.declare_parameter("basics", True).value
        self.test_wheels = self.declare_parameter("wheels", False).value
        self.test_dock = self.declare_parameter("dock", False).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.stamped_cmd_vel = self.declare_parameter("stamped_cmd_vel", True).value

        print(f"\n{Logger.BOLD}Starting Navigation Subtask \n")

        self.navigation_manager = NavigationTasks(
            self, task=self.task_to_test, mock_data=self.mocked
        )

        cmd_vel_type = TwistStamped if self.stamped_cmd_vel else Twist
        self.cmd_vel_pub = self.create_publisher(cmd_vel_type, self.cmd_vel_topic, 10)
        self.readings = []

        self.basic_funcs = {}
        if self.run_basics and not self.mocked:
            self.basic_funcs = {
                "Base Motors": {"func": self.check_motors},
                "Bus Voltage": {"func": self.check_bus_voltage},
                "Lidar": {"func": self.check_scan},
                "Localization (TF map -> base_link)": {"func": self.check_localization},
                "Nav Services": {"func": self.check_services},
            }
            if self.test_wheels:
                self.basic_funcs["Wheels"] = {"func": self.check_wheels}

        self.tests_funcs = {
            "Check Door": {"func": self.navigation_manager.check_door},
            "Retrieve Areas": {"func": self.navigation_manager.retrieve_areas},
            "Get Current Pose": {"func": self.navigation_manager.get_current_pose},
            "Move to Location": {
                "func": self.navigation_manager.move_to_location,
                "location": "entrance",
                "sublocation": "",
            },
            "Follow Person Start": {
                "func": self.navigation_manager.follow_person,
                "follow": True,
            },
            "Follow Person Stop": {
                "func": self.navigation_manager.follow_person,
                "follow": False,
            },
            "Get Path Info": {
                "func": self.navigation_manager.get_path_info,
                "location_b": "entrance",
                "sublocation_b": "",
            },
            # Approaches a virtual target 1.2 m ahead: with standoff 0.5 the
            # robot should advance ~0.7 m and stop facing the point (safe to
            # run in an open area; nav_central snaps to a costmap-free pose).
            "Approach Point (1.2m ahead)": {
                "func": self.navigation_manager.approach_point,
                "point": _front_point(1.2),
                "standoff": 0.5,
            },
            "Explore Zone": {
                "func": self.navigation_manager.explore_zone,
                "step": EXPLORE_STEP,
            },
            "Return to Origin": {"func": self.navigation_manager.return_to_origin},
        }
        if self.test_dock:
            self.tests_funcs["Dock Table"] = {
                "func": self.navigation_manager.dock_table,
                "offset": 0.0,
            }

        print(f"\n{Logger.BOLD}Testing {len(self.tests_funcs)} available subtaks..... \n")
        self.run()

    def collect(self, topic, msg_type, count=1):
        msgs = []
        sub = self.create_subscription(
            msg_type, topic, lambda m: msgs.append(m), qos_profile_sensor_data
        )
        deadline = time.time() + TOPIC_TIMEOUT
        while len(msgs) < count and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        assert msgs, f"No messages on {topic} after {TOPIC_TIMEOUT:.0f}s"
        return msgs

    def collect_axes(self, topic, msg_type):
        data = list(self.collect(topic, msg_type)[-1].data)
        assert len(data) >= NUM_WHEELS, f"{topic}: {len(data)} axes, expected {NUM_WHEELS}"
        return data[:NUM_WHEELS]

    def publish_twist(self, vx, vy, wz):
        if self.stamped_cmd_vel:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def check_motors(self):
        errors = [int(e) for e in self.collect_axes(AXIS_ERRORS_TOPIC, Int32MultiArray)]
        states = [int(s) for s in self.collect_axes(AXIS_STATES_TOPIC, Int32MultiArray)]
        self.readings.append(f"  axis errors : {[hex(e) for e in errors]}")
        self.readings.append(f"  axis states : {states} (8 = closed loop)")
        assert not any(errors), f"Axes with errors: {[hex(e) for e in errors]}"
        bad = {i: s for i, s in enumerate(states) if s != AXIS_STATE_CLOSED_LOOP}
        assert not bad, f"Axes not in closed loop, they ignore cmd_vel: {bad}"

    def check_bus_voltage(self):
        volts = self.collect_axes(BUS_VOLTAGE_TOPIC, Float32MultiArray)
        self.readings.append(f"  bus voltage : {[round(v, 2) for v in volts]} V")
        low = {i: round(v, 2) for i, v in enumerate(volts) if v < MIN_BUS_VOLTAGE}
        assert not low, f"Bus voltage below {MIN_BUS_VOLTAGE} V: {low}"

    def check_wheels(self):
        """Spin in place and check every wheel turns, and all the same way."""
        peaks = [0.0] * NUM_WHEELS
        signs = [0.0] * NUM_WHEELS

        def on_vel(msg):
            for i, v in enumerate(list(msg.data)[:NUM_WHEELS]):
                if abs(v) > peaks[i]:
                    peaks[i] = abs(v)
                    signs[i] = math.copysign(1.0, v)

        sub = self.create_subscription(
            Float32MultiArray, WHEEL_VEL_TOPIC, on_vel, qos_profile_sensor_data
        )
        try:
            deadline = time.time() + WHEEL_TEST_TIME
            while time.time() < deadline:
                self.publish_twist(0.0, 0.0, WHEEL_TEST_WZ)
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            for _ in range(10):
                self.publish_twist(0.0, 0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
            self.destroy_subscription(sub)

        self.readings.append(f"  wheel peaks : {[round(p, 3) for p in peaks]}")
        assert any(peaks), f"No feedback on {WHEEL_VEL_TOPIC}, is the base moving?"
        dead = {i: round(p, 3) for i, p in enumerate(peaks) if p < WHEEL_MIN_VEL}
        assert not dead, f"Wheels that did not spin: {dead}"
        # A pure rotation turns all four the same way, a different sign means
        # that motor or its encoder is reversed
        assert len(set(signs)) == 1, f"Wheel signs do not match: {signs}"

    def check_scan(self):
        scan = self.collect(SCAN_TOPIC, LaserScan, count=3)[-1]
        valid = [
            r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max
        ]
        ratio = len(valid) / max(len(scan.ranges), 1)
        self.readings.append(
            f"  scan        : {len(scan.ranges)} beams, {ratio:.0%} valid, {scan.header.frame_id}"
        )
        assert ratio > 0.1, f"Only {ratio:.0%} valid readings, a lidar is blocked or stopped"

    def check_localization(self):
        buffer = self.navigation_manager.tf_buffer
        deadline = time.time() + TOPIC_TIMEOUT
        while time.time() < deadline:
            if buffer.can_transform("map", "base_link", rclpy.time.Time()):
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise AssertionError("No TF map -> base_link, set the initial pose in nav_ui")

    def check_services(self):
        nav = self.navigation_manager
        clients = [
            nav.door_checking_srv,
            nav.retrieve_areas_srv,
            nav.move_to_location_srv,
            nav.follow_person_srv,
            nav.nav_query_srv,
            nav.dock_table_srv,
            nav.go_to_pose_srv,
            nav.get_robot_pose_srv,
            nav.approach_point_srv,
        ]
        missing = [c.srv_name for c in clients if not c.wait_for_service(timeout_sec=2.0)]
        assert not missing, f"Services down: {missing}"

    def check_basic(self, func, *args, **kwargs):
        func(**kwargs)

    def check_nav_task(self, func, *args, **kwargs):
        result = func(**kwargs)
        # Check for map_service case
        if (
            result[0] == Status.EXECUTION_ERROR
            and result[1] == self.navigation_manager.areas_backup
        ):
            assert False, "Service not started or Service return empty"

        assert result[0] == Status.EXECUTION_SUCCESS, result[1]

    def run(self):
        passed = 0
        failed = 0
        if self.test_wheels and not self.mocked:
            print(
                f"  {Logger.RED}{Logger.BOLD}Wheel test moves the robot, "
                f"starting in 5s...{Logger.RESET}"
            )
            deadline = time.time() + 5.0
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        for checker, tests in (
            (self.check_basic, self.basic_funcs),
            (self.check_nav_task, self.tests_funcs),
        ):
            for command in tests:
                if Logger.run_test(
                    f"Test - {command}",
                    checker,
                    **tests[command],
                    clear_logs=self.logs,
                ):
                    passed += 1
                else:
                    failed += 1

        if self.readings:
            print(f"\n{Logger.BOLD}Readings:{Logger.RESET}")
            for line in self.readings:
                print(line)
        print()
        if failed == 0:
            print(f"  {Logger.GREEN}{Logger.BOLD}All {passed} tests passed!{Logger.RESET}\n")
        else:
            print(
                f"  {Logger.GREEN}{passed} passed{Logger.RESET}, {Logger.RED}{failed} failed{Logger.RESET}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TestNavigationManager()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
