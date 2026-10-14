#!/usr/bin/env python3
"""Generic spec-driven task manager.

One node for any task: the steps come from a YAML spec and the recovery policy
is selectable, which is what the A/B against the hand-written FSMs needs.

    ros2 run task_manager agentic_task_manager.py --ros-args \
        -p spec:=safety -p policy:=llm \
        -p base_url:=http://<hri-orin>:11434/v1 -p model:=frida-llm
"""

import json
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from task_manager.agentic.executor import TaskExecutor
from task_manager.agentic.ros_skills import build_ros_skills
from task_manager.agentic.spec import load_spec
from task_manager.agentic.supervisor import make_supervisor
from task_manager.utils.logger import Logger
from task_manager.utils.subtask_manager import SubtaskManager, Task


def resolve_spec_path(spec: str) -> str:
    if os.path.isfile(spec):
        return spec
    share = get_package_share_directory("task_manager")
    return os.path.join(share, "config", "agentic", f"{spec}.yaml")


class AgenticTaskManager(Node):
    def __init__(self):
        super().__init__("agentic_task_manager")
        self.declare_parameter("spec", "safety")
        self.declare_parameter("policy", "rules")  # rules | rules+ | llm
        self.declare_parameter("base_url", "http://localhost:11434/v1")
        self.declare_parameter("model", "frida-llm")
        self.declare_parameter("llm_timeout_s", 5.0)
        self.declare_parameter("task", "DEBUG")  # frida Task enum name for SubtaskManager
        self.declare_parameter("results_dir", "/tmp/agentic_runs")

        spec_path = resolve_spec_path(self.get_parameter("spec").value)
        self.spec = load_spec(spec_path)
        policy = self.get_parameter("policy").value
        self.supervisor = make_supervisor(
            policy,
            permanent_errors=self.spec.permanent_errors,
            base_url=self.get_parameter("base_url").value,
            model=self.get_parameter("model").value,
            persona=self.spec.persona,
            rules=self.spec.rules,
            timeout_s=float(self.get_parameter("llm_timeout_s").value),
        )
        self.subtask_manager = SubtaskManager(
            self, task=Task[self.get_parameter("task").value], mock_areas=[]
        )
        self.skills = build_ros_skills(self, self.subtask_manager)
        Logger.info(self, f"Agentic task manager: spec={spec_path} policy={policy}")

    def run(self):
        executor = TaskExecutor(
            self.spec, self.skills, self.supervisor, log=lambda m: Logger.info(self, m)
        )
        result = executor.run()
        Logger.success(
            self,
            f"{self.spec.name} finished: {result.points:.0f}/{result.max_points:.0f} points "
            f"in {result.elapsed_s:.0f}s (fallbacks={result.fallbacks_run}, "
            f"invalid_llm={result.invalid_decisions})",
        )
        self._save(result)

    def _save(self, result):
        """Persist each run so robot A/B trials can be compared offline."""
        out_dir = self.get_parameter("results_dir").value
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f"{self.spec.name}_{result.policy}_{int(time.time())}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "task": result.task,
                    "policy": result.policy,
                    "points": result.points,
                    "max_points": result.max_points,
                    "elapsed_s": result.elapsed_s,
                    "timed_out": result.timed_out,
                    "aborted": result.aborted,
                    "outcomes": result.outcomes,
                    "fallbacks": result.fallbacks_run,
                    "decisions": [d.__dict__ for d in result.decisions],
                    "utterances": result.utterances,
                    "events": result.events,
                },
                f,
                indent=1,
            )
        Logger.info(self, f"Run saved to {path}")


def main(args=None):
    rclpy.init(args=args)
    node = AgenticTaskManager()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
