#!/usr/bin/env python3
"""Runs one seeded scenario: sim stack, real task manager, referee, score and report.

    ros2 run frida_task_sim run_scenario.py --task ppc --seed 1
    ros2 run frida_task_sim run_scenario.py --scenario <run_dir>/scenario.json

Exit code: 0 scored run, 2 infrastructure failure (stack not ready or task manager crash).
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from frida_constants.hri_constants import START_BUTTON_CLIENT
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from frida_task_sim import (
    catalog,
    report,
    scenario as scenarios,
    scoring,
    venue as venues,
)
from frida_task_sim import world as worlds
from frida_task_sim.timeline import BOLD, DIM, RESET, Timeline

READY_TIMEOUT = 420
# Extra sim seconds after the rulebook limit before the task manager is stopped
GRACE = 20

TASK_MANAGERS = {
    "ppc": ("pickandplace_task_manager.py", "PPC ready"),
}


class Stack:
    """Background processes of one run, each logging to its own file."""

    def __init__(self, log_dir: str, env: dict):
        self.log_dir = log_dir
        self.env = env
        self.processes: dict[str, subprocess.Popen] = {}
        os.makedirs(log_dir, exist_ok=True)

    def log(self, name: str) -> str:
        return os.path.join(self.log_dir, f"{name}.log")

    def start(self, name: str, cmd: list[str]):
        out = open(self.log(name), "w")
        self.processes[name] = subprocess.Popen(
            cmd,
            stdout=out,
            stderr=subprocess.STDOUT,
            env=self.env,
            start_new_session=True,
        )

    def wait_for(self, name: str, *patterns: str, timeout: float = READY_TIMEOUT):
        deadline = time.time() + timeout
        for pattern in patterns:
            while True:
                with open(self.log(name), errors="replace") as f:
                    if pattern in f.read():
                        break
                if self.processes[name].poll() is not None:
                    raise RuntimeError(
                        f"{name} exited before '{pattern}' (see {self.log(name)})"
                    )
                if time.time() > deadline:
                    raise RuntimeError(
                        f"timed out waiting for '{pattern}' in {self.log(name)}"
                    )
                time.sleep(1.0)

    def stop(self, name: str, grace: float = 8.0):
        process = self.processes.pop(name, None)
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)

    def stop_all(self):
        for name in reversed(list(self.processes)):
            self.stop(name)
        # gz and nav2 leave children behind that hold the DDS endpoints of the next run
        patterns = ("gz sim", "/workspace/install/", "/opt/ros/jazzy/lib/")
        for pid in filter(str.isdigit, os.listdir("/proc")):
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if int(pid) != os.getpid() and "run_scenario" not in cmdline:
                if any(pattern in cmdline for pattern in patterns):
                    os.kill(int(pid), signal.SIGKILL)


class Monitor(Node):
    """Prints the live timeline and watches the sim clock."""

    def __init__(self, scenario: dict):
        super().__init__(
            "task_sim_runner",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.timeline = Timeline(scenario)
        self.started_at = None
        self.button = self.create_publisher(Empty, START_BUTTON_CLIENT, 10)
        self.finish = self.create_client(Trigger, "/sim/referee/finish")
        self.create_subscription(String, "/sim/referee/events", self._on_event, 100)

    def _on_event(self, msg: String):
        event = json.loads(msg.data)
        if event["type"] == "start":
            self.started_at = self.get_clock().now()
        for colour, text in self.timeline.add(event):
            print(f"{colour}{text}{RESET}", flush=True)

    def elapsed(self) -> float:
        if self.started_at is None:
            return 0.0
        return (self.get_clock().now() - self.started_at).nanoseconds / 1e9


def prepare(args) -> tuple[dict, str]:
    """Scenario plus a run directory holding everything the stack reads."""
    if args.scenario:
        scenario = scenarios.load(args.scenario)
    else:
        scenario = scenarios.generate(args.task, args.seed)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = args.out or os.path.join(
        args.runs_dir, f"{scenario['task']}_seed{scenario['seed']}_{stamp}"
    )
    os.makedirs(run_dir, exist_ok=True)
    scenarios.save(scenario, os.path.join(run_dir, "scenario.json"))
    with open(os.path.join(run_dir, "world.sdf"), "w") as f:
        f.write(worlds.build(scenario))
    grasp = {
        "world": worlds.WORLD_NAME,
        "objects": {
            o["id"]: [catalog.height(o["name"]), catalog.radius(o["name"])]
            for o in scenario["objects"]
        },
    }
    with open(os.path.join(run_dir, "grasp_objects.json"), "w") as f:
        json.dump(grasp, f)
    venue = venues.load(scenario["venue"])
    cabinet = venue["furniture"][venue["ppc"]["cabinet"]]
    shelf = {
        "frame": "base_link",
        "compartment_height": 0.26,
        "arenas": {"1": cabinet["levels"]},
    }
    with open(os.path.join(run_dir, "shelf_levels.json"), "w") as f:
        json.dump(shelf, f)
    # slam_toolbox starts localizing from the spawn pose
    gz_share = get_package_share_directory("frida_gz_sim")
    with open(os.path.join(gz_share, "config", "mapper_params_sim.yaml")) as f:
        mapper = yaml.safe_load(f)
    x, y, _, yaw = venue["spawn"]
    mapper["slam_toolbox"]["ros__parameters"]["map_start_pose"] = [x, y, yaw]
    with open(os.path.join(run_dir, "mapper_params.yaml"), "w") as f:
        yaml.safe_dump(mapper, f)
    return scenario, run_dir


def start_stack(stack: Stack, scenario: dict, run_dir: str, gui: bool):
    venue = venues.load(scenario["venue"])
    x, y, z, yaw = venue["spawn"]
    print(f"{DIM}starting Gazebo...{RESET}", flush=True)
    stack.start(
        "sim",
        [
            "ros2",
            "launch",
            "frida_gz_sim",
            "sim.launch.py",
            f"gui:={str(gui).lower()}",
            f"world:={run_dir}/world.sdf",
            "mobile_base:=true",
            "grasp_assist:=true",
            "camera_rate:=5",
            f"objects_file:={run_dir}/grasp_objects.json",
            f"spawn_x:={x}",
            f"spawn_y:={y}",
            f"spawn_z:={z}",
            f"spawn_yaw:={yaw}",
        ],
    )
    stack.wait_for("sim", "activated xarm6_traj_controller", "Grasp attach ready")
    print(
        f"{DIM}starting manipulation, navigation and the harness...{RESET}", flush=True
    )
    stack.start(
        "manipulation", ["ros2", "launch", "frida_gz_sim", "sim_manipulation.launch.py"]
    )
    stack.start(
        "navigation",
        [
            "ros2",
            "launch",
            "frida_gz_sim",
            "sim_nav.launch.py",
            f"map_name:={venue['map_name']}",
            f"mapper_params_file:={run_dir}/mapper_params.yaml",
            f"spawn_x:={x}",
            f"spawn_y:={y}",
            f"spawn_yaw:={yaw}",
        ],
    )
    stack.start(
        "harness",
        [
            "ros2",
            "launch",
            "frida_task_sim",
            "task_sim.launch.py",
            f"scenario_file:={run_dir}/scenario.json",
            f"run_dir:={run_dir}",
        ],
    )
    stack.wait_for("manipulation", "Manipulation core started")
    stack.wait_for("navigation", "Finished Setup, Starting monitoring")
    stack.wait_for("harness", "Referee ready", "Ground-truth detector ready")


def task_manager_cmd(scenario: dict, run_dir: str) -> list[str]:
    share = get_package_share_directory("frida_task_sim")
    script, _ = TASK_MANAGERS[scenario["task"]]
    cmd = [
        "ros2",
        "run",
        "task_manager",
        script,
        "--ros-args",
        "-p",
        "use_sim_time:=true",
    ]
    if scenario["task"] == "ppc":
        cmd += [
            "-p",
            f"spec_file:={share}/config/tasks/ppc_sim.json",
            "-p",
            f"log_dir:={run_dir}/bt",
        ]
    return cmd


def run(args) -> int:
    scenario, run_dir = prepare(args)
    print(
        f"{BOLD}{scenario['task'].upper()} seed {scenario['seed']} -> {run_dir}{RESET}",
        flush=True,
    )
    env = dict(
        os.environ,
        FRIDA_SHELF_LEVELS_FILE=f"{run_dir}/shelf_levels.json",
        FRIDA_ARENA="1",
        PYTHONUNBUFFERED="1",
    )
    stack = Stack(os.path.join(run_dir, "stack_logs"), env)
    rclpy.init()
    monitor = Monitor(scenario)
    status, code = "completed", 0
    try:
        start_stack(stack, scenario, run_dir, args.gui)
        _, ready = TASK_MANAGERS[scenario["task"]]
        stack.log_dir = run_dir
        stack.start("tm", task_manager_cmd(scenario, run_dir))
        stack.wait_for("tm", ready, timeout=120)
        tm = stack.processes["tm"]
        limit = scenario["time_limit_s"] + GRACE
        last_press = 0.0
        while tm.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            if monitor.started_at is None and time.time() - last_press > 2.0:
                monitor.button.publish(Empty())
                last_press = time.time()
            if monitor.elapsed() > limit:
                status = "stopped at the time limit"
                break
        if tm.poll() not in (None, 0):
            status, code = f"task manager crashed (exit {tm.poll()})", 2
        stack.stop("tm")
        if monitor.finish.wait_for_service(timeout_sec=5.0):
            future = monitor.finish.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(monitor, future, timeout_sec=10.0)
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(monitor, timeout_sec=0.1)
    except (RuntimeError, KeyboardInterrupt) as e:
        status, code = f"infrastructure failure: {e or 'interrupted'}", 2
        print(f"\033[31m{status}{RESET}", flush=True)
    finally:
        stack.stop_all()
        monitor.destroy_node()
        rclpy.try_shutdown()

    final = {}
    final_path = os.path.join(run_dir, "final.json")
    if os.path.exists(final_path):
        with open(final_path) as f:
            final = json.load(f)
    events = monitor.timeline.events
    score = scoring.score(scenario, events, final)
    score["status"] = status
    with open(os.path.join(run_dir, "score.json"), "w") as f:
        json.dump(score, f, indent=2)
    report.write(run_dir, scenario, score, events, status)
    print(
        f"\n{BOLD}===== {scenario['task'].upper()} seed {scenario['seed']}: {status} ====={RESET}"
    )
    for row in score["rows"]:
        if row["simulated"] and row["scored"]:
            print(f"  {row['total']:+5d}  {row['scored']}x {row['label']}")
    print(
        f"{BOLD}  SCORE {score['total']} / {score['max_in_sim']} reachable in sim "
        f"({score['max_rulebook']} on the rulebook sheet){RESET}"
    )
    print(f"  report: {run_dir}/report.html")
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", default="ppc", choices=sorted(TASK_MANAGERS))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--scenario", help="replay a saved scenario.json")
    parser.add_argument(
        "--out", help="run directory (default: <runs-dir>/<task>_seed<N>_<time>)"
    )
    parser.add_argument("--runs-dir", default="/workspace/sim_logs/runs")
    parser.add_argument("--gui", action="store_true", help="open the Gazebo window")
    sys.exit(run(parser.parse_args()))


if __name__ == "__main__":
    main()
